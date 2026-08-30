package vive

import (
	"context"
	"fmt"
	"sync"
	"time"

	commonpb "go.viam.com/api/common/v1"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/resource"
	"go.viam.com/rdk/services/generic"
	"go.viam.com/rdk/services/worldstatestore"
)

var _ worldstatestore.Service = (*visualizer)(nil)

func init() {
	resource.RegisterService(worldstatestore.API, TeleopVisualizer,
		resource.Registration[worldstatestore.Service, *VisualizerConfig]{
			Constructor: newVisualizer,
		},
	)
}

const (
	defaultPublishRateHz = 20.0
	defaultStaleAfterMS  = 500
)

// Validate declares the teleop service dependency.
//
// It deliberately does NOT set defaults. In a module, viam-server decodes a
// fresh config for the constructor and never calls Validate on that instance
// (module/resources.go:37-45) — only ValidateConfig runs, on a different object,
// and only its dependency strings cross the wire. Defaults applied here would
// silently fail to arrive: a zero PublishRateHz makes the ticker interval
// MaxInt64 so nothing is ever published, and a zero StaleAfterMS makes every
// hand instantly stale so no controller ever renders. Both are silent, which is
// the failure class this tool exists to eliminate. Defaults live in
// applyDefaults, called from the constructor.
func (cfg *VisualizerConfig) Validate(path string) ([]string, []string, error) {
	if cfg.TeleopService == "" {
		return nil, nil, fmt.Errorf("%s: teleop_service is required", path)
	}
	return []string{cfg.TeleopService}, nil, nil
}

// applyDefaults fills unset fields. Called from the constructor, which is the
// only place guaranteed to see the config the resource actually runs with.
func (cfg *VisualizerConfig) applyDefaults() {
	if cfg.PublishRateHz <= 0 {
		cfg.PublishRateHz = defaultPublishRateHz
	}
	if cfg.StaleAfterMS <= 0 {
		cfg.StaleAfterMS = defaultStaleAfterMS
	}
}

type visualizer struct {
	resource.AlwaysRebuild // so viam-server re-resolves the teleop handle on rebuild

	name   resource.Name
	logger logging.Logger
	cfg    *VisualizerConfig
	teleop *teleopService

	cancel context.CancelFunc
	done   chan struct{}

	mu      sync.RWMutex
	current map[string]*commonpb.Transform // last built state: current truth
	changes chan worldstatestore.TransformChange
	closed  bool // set by Close; stops a later stream starting an orphan publisher
	reseed  bool // set when a stream opens; makes run() re-ADD everything
}

func newVisualizer(
	ctx context.Context,
	deps resource.Dependencies,
	rawConf resource.Config,
	logger logging.Logger,
) (worldstatestore.Service, error) {
	cfg, err := resource.NativeConfig[*VisualizerConfig](rawConf)
	if err != nil {
		return nil, err
	}
	cfg.applyDefaults() // must happen here; see the comment on Validate

	raw, err := generic.FromProvider(deps, cfg.TeleopService)
	if err != nil {
		return nil, err
	}
	// VisualState is a method on *teleopService, not on generic.Service. This
	// assertion succeeds only because RDK resolves module-local dependencies to
	// the real in-process object rather than a gRPC client. Fail hard: a
	// visualizer that silently published nothing would be indistinguishable from
	// one whose robot is idle.
	svc, ok := raw.(*teleopService)
	if !ok {
		return nil, fmt.Errorf(
			"teleop_service %q did not resolve to an in-process teleop service (got %T); "+
				"the visualizer must be configured in the same module as the teleop service",
			cfg.TeleopService, raw)
	}

	v := &visualizer{
		name:    rawConf.ResourceName(),
		logger:  logger,
		cfg:     cfg,
		teleop:  svc,
		current: map[string]*commonpb.Transform{},
		// Buffered so a slow consumer is dropped rather than stalling the publisher.
		changes: make(chan worldstatestore.TransformChange, 256),
		done:    make(chan struct{}),
	}
	return v, nil
}

func (v *visualizer) Name() resource.Name { return v.name }

// Status is required by resource.Resource. resource.AlwaysRebuild supplies only
// Reconfigure, so this must be defined explicitly — all three existing models in
// this repo do the same (teleop.go, capture_control.go, module.go).
func (v *visualizer) Status(ctx context.Context) (map[string]interface{}, error) {
	v.mu.RLock()
	defer v.mu.RUnlock()
	return map[string]interface{}{"transforms": len(v.current)}, nil
}

// ListUUIDs and GetTransform serve `current` — present truth, not delivered
// state. With conditional advancement the two can transiently differ, and a
// reconnecting client seeding itself should get truth.
func (v *visualizer) ListUUIDs(ctx context.Context, extra map[string]any) ([][]byte, error) {
	v.mu.RLock()
	defer v.mu.RUnlock()

	out := make([][]byte, 0, len(v.current))
	for _, t := range v.current {
		out = append(out, t.Uuid)
	}
	return out, nil
}

func (v *visualizer) GetTransform(
	ctx context.Context, uuid []byte, extra map[string]any,
) (*commonpb.Transform, error) {
	v.mu.RLock()
	defer v.mu.RUnlock()

	if t, ok := v.current[string(uuid)]; ok {
		return t, nil
	}
	return nil, fmt.Errorf("no transform with uuid %q", uuid)
}

func (v *visualizer) DoCommand(ctx context.Context, cmd map[string]interface{}) (map[string]interface{}, error) {
	return nil, resource.ErrDoUnimplemented
}

func (v *visualizer) Close(ctx context.Context) error {
	// v.cancel is written by StreamTransformChanges under v.mu, and viam-server
	// closes resources from a different goroutine than the gRPC handler, so this
	// read must be guarded. Release the lock before waiting, or run() cannot take
	// it to update v.current and Close deadlocks.
	v.mu.Lock()
	v.closed = true
	cancel, done := v.cancel, v.done
	v.mu.Unlock()

	if cancel != nil {
		cancel()
		<-done
	}
	return nil
}

// StreamTransformChanges returns the change stream. One shared channel serves all
// callers, so two simultaneous viewers split changes between them — the RDK fake
// has the same limitation, and ListUUIDs/GetTransform still return correct state
// to both. Starts the publisher on first use.
func (v *visualizer) StreamTransformChanges(
	ctx context.Context, extra map[string]any,
) (*worldstatestore.TransformChangeStream, error) {
	v.mu.Lock()
	defer v.mu.Unlock()

	// Close may already have run: viam-server closes resources on reconfigure
	// while gRPC handlers are still in flight. Without this check, a stream
	// arriving just after Close would start a publisher on a context nothing can
	// cancel, leaking a goroutine that polls a dead teleop service forever.
	if v.closed {
		return nil, fmt.Errorf("%s: visualizer is closed", v.name)
	}

	// Re-send everything to this subscriber. `delivered` is per-resource, not
	// per-stream, so a viewer that reopens the scene would otherwise receive
	// nothing: its objects are already in `delivered`, and a stationary frame like
	// vive-lighthouse emits no UPDATED either, leaving the scene empty. Relying on
	// the client to seed itself via ListUUIDs/GetTransform is an unverified
	// assumption about the app, and the cost of being wrong is a blank scene.
	v.reseed = true

	if v.cancel == nil {
		runCtx, cancel := context.WithCancel(context.Background())
		v.cancel = cancel
		go v.run(runCtx)
	}

	return worldstatestore.NewTransformChangeStreamFromChannel(ctx, v.changes), nil
}

func (v *visualizer) run(ctx context.Context) {
	defer close(v.done)

	ticker := time.NewTicker(time.Duration(float64(time.Second) / v.cfg.PublishRateHz))
	defer ticker.Stop()

	// delivered is what the consumer is known to have. It advances only for
	// changes actually enqueued — see applyCommitted.
	delivered := map[string]*commonpb.Transform{}

	for {
		select {
		case <-ctx.Done():
			return
		case <-ticker.C:
		}

		var next map[string]*commonpb.Transform
		st, ok := v.teleop.VisualState()
		if ok {
			next = buildTransforms(st, *v.cfg, time.Now())
		} else {
			// The teleop service has been closed and replaced. Publish an empty
			// world so the app clears, rather than freezing stale geometry.
			next = map[string]*commonpb.Transform{}
		}

		v.mu.Lock()
		v.current = next
		if v.reseed {
			// A new subscriber arrived. Forget what the previous one had so every
			// object is re-ADDED for this one.
			v.reseed = false
			delivered = map[string]*commonpb.Transform{}
		}
		v.mu.Unlock()

		changes := diffTransforms(delivered, next)
		if len(changes) == 0 {
			continue
		}

		sent := make([]bool, len(changes))
		for i, c := range changes {
			select {
			case v.changes <- c:
				sent[i] = true
			default:
				// Consumer is slow or gone. Drop, but do not advance delivered for
				// this change, so it is re-emitted next tick.
			}
		}
		delivered = applyCommitted(delivered, next, changes, sent)
	}
}
