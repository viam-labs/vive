package vive

import (
	"context"
	"fmt"
	"os"
	"sync"
	"time"

	datapb "go.viam.com/api/app/data/v1"
	"go.viam.com/rdk/components/sensor"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/resource"
	rutils "go.viam.com/rdk/utils"
	"go.viam.com/utils/rpc"
	"google.golang.org/protobuf/types/known/timestamppb"
)

func init() {
	resource.RegisterComponent(sensor.API, CaptureControl,
		resource.Registration[sensor.Sensor, *CaptureControlConfig]{
			Constructor: newCaptureControl,
		},
	)
}

type CaptureControlConfig struct {
	ArmName            string   `json:"arm_name,omitempty"`
	ArmNames           []string `json:"arm_names,omitempty"`
	CameraNames        []string `json:"camera_names,omitempty"`
	GripperNames       []string `json:"gripper_names,omitempty"`
	CaptureFrequencyHz float64  `json:"capture_frequency_hz,omitempty"`

	// Sequence creation. Credentials and part ID default to the VIAM_API_KEY,
	// VIAM_API_KEY_ID, and VIAM_MACHINE_PART_ID env vars the module manager injects.
	DisableSequences bool   `json:"disable_sequences,omitempty"`
	PartID           string `json:"part_id,omitempty"`
	APIKey           string `json:"api_key,omitempty"`
	APIKeyID         string `json:"api_key_id,omitempty"`
}

func (cfg *CaptureControlConfig) arms() []string {
	arms := cfg.ArmNames
	if cfg.ArmName != "" {
		arms = append([]string{cfg.ArmName}, arms...)
	}
	return arms
}

type resourceMethod struct {
	resource string
	method   string
}

func (cfg *CaptureControlConfig) resourceMethods() []resourceMethod {
	var rms []resourceMethod
	for _, armName := range cfg.arms() {
		rms = append(rms, resourceMethod{armName, "EndPosition"}, resourceMethod{armName, "JointPositions"})
	}
	for _, camName := range cfg.CameraNames {
		rms = append(rms, resourceMethod{camName, "GetImages"})
	}
	for _, gripperName := range cfg.GripperNames {
		rms = append(rms, resourceMethod{gripperName, "DoCommand"})
	}
	return rms
}

func (cfg *CaptureControlConfig) Validate(path string) ([]string, []string, error) {
	if len(cfg.arms()) == 0 {
		return nil, nil, fmt.Errorf("%s: arm_name or arm_names is required", path)
	}
	return nil, nil, nil
}

type captureControl struct {
	resource.AlwaysRebuild

	name   resource.Name
	logger logging.Logger
	cfg    *CaptureControlConfig

	// sequenceFn creates the cloud sequence for a finished session. Indirected so
	// tests can observe the window without dialing app.viam.com.
	sequenceFn func(start, end time.Time, tags []string)

	mu            sync.RWMutex
	capturing     bool
	captureFreqHz float64
	task          string
	sessionTags   []string
	sessionStart  time.Time
	activeHands   map[string]struct{}
	activeGrips   int
}

func newCaptureControl(ctx context.Context, deps resource.Dependencies, rawConf resource.Config, logger logging.Logger) (sensor.Sensor, error) {
	conf, err := resource.NativeConfig[*CaptureControlConfig](rawConf)
	if err != nil {
		return nil, err
	}

	freqHz := conf.CaptureFrequencyHz
	if freqHz <= 0 {
		freqHz = 10.0
	}

	cc := &captureControl{
		name:          rawConf.ResourceName(),
		logger:        logger,
		cfg:           conf,
		captureFreqHz: freqHz,
		activeHands:   make(map[string]struct{}),
	}
	cc.sequenceFn = cc.createSequence
	return cc, nil
}

func (cc *captureControl) Name() resource.Name {
	return cc.name
}

func (cc *captureControl) Status(ctx context.Context) (map[string]interface{}, error) {
	return map[string]interface{}{}, nil
}

func (cc *captureControl) Readings(ctx context.Context, extra map[string]interface{}) (map[string]interface{}, error) {
	cc.mu.RLock()
	defer cc.mu.RUnlock()

	freq := 0.0
	var tags []interface{}
	if cc.capturing {
		freq = cc.captureFreqHz
		for _, t := range cc.sessionTags {
			tags = append(tags, t)
		}
	}

	var overrides []interface{}
	for _, rm := range cc.cfg.resourceMethods() {
		overrides = append(overrides, map[string]interface{}{
			"resource_name":        rm.resource,
			"method":               rm.method,
			"capture_frequency_hz": freq,
			"tags":                 tags,
		})
	}

	return map[string]interface{}{
		"overrides": overrides,
	}, nil
}

func (cc *captureControl) DoCommand(ctx context.Context, cmd map[string]interface{}) (map[string]interface{}, error) {
	if _, ok := cmd["start-capture"]; ok {
		cc.mu.Lock()
		defer cc.mu.Unlock()

		cc.activeGrips++
		if cc.activeGrips == 1 {
			cc.capturing = true
			cc.sessionStart = time.Now()
			sessionTag := fmt.Sprintf("session:%s", cc.sessionStart.Format("20060102_150405"))
			cc.sessionTags = []string{sessionTag}
			if cc.task != "" {
				cc.sessionTags = append(cc.sessionTags, fmt.Sprintf("cmd:%s", cc.task))
			}
			cc.logger.Infof("capture started: tags=%v freq=%.1fHz", cc.sessionTags, cc.captureFreqHz)
		}

		return map[string]interface{}{
			"capturing":    true,
			"active_grips": cc.activeGrips,
			"tags":         cc.sessionTags,
		}, nil
	}

	if _, ok := cmd["stop-capture"]; ok {
		cc.mu.Lock()
		if cc.activeGrips > 0 {
			cc.activeGrips--
		}
		// Another hand is still driving — keep the session open.
		if cc.activeGrips > 0 {
			grips := cc.activeGrips
			cc.mu.Unlock()
			return map[string]interface{}{
				"capturing":    true,
				"active_grips": grips,
			}, nil
		}

		cc.capturing = false
		start := cc.sessionStart
		tags := cc.sessionTags
		cc.sessionTags = nil
		cc.sessionStart = time.Time{}
		cc.mu.Unlock()

		cc.logger.Info("capture stopped")

		if !cc.cfg.DisableSequences && !start.IsZero() {
			end := time.Now()
			go cc.sequenceFn(start, end, tags)
		}

		return map[string]interface{}{
			"capturing":    false,
			"active_grips": 0,
		}, nil
	}

	if taskCmd, ok := cmd["set_task"]; ok {
		cc.mu.Lock()
		defer cc.mu.Unlock()

		taskStr, _ := taskCmd.(string)
		cc.task = taskStr
		cc.logger.Infof("task set: %q", taskStr)
		return map[string]interface{}{
			"task": taskStr,
		}, nil
	}

	if _, ok := cmd["status"]; ok {
		cc.mu.RLock()
		defer cc.mu.RUnlock()

		return map[string]interface{}{
			"capturing":            cc.capturing,
			"capture_frequency_hz": cc.captureFreqHz,
			"active_grips":         cc.activeGrips,
			"tags":                 cc.sessionTags,
			"task":                 cc.task,
		}, nil
	}

	return nil, fmt.Errorf("unknown command: %v", cmd)
}

func firstNonEmpty(vals ...string) string {
	for _, v := range vals {
		if v != "" {
			return v
		}
	}
	return ""
}

// createSequence registers the just-completed teleop window as a sequence in the
// cloud Data API. rdk's app client does not expose sequences, so we dial directly.
func (cc *captureControl) createSequence(start, end time.Time, tags []string) {
	partID := firstNonEmpty(cc.cfg.PartID, os.Getenv(rutils.MachinePartIDEnvVar))
	apiKey := firstNonEmpty(cc.cfg.APIKey, os.Getenv(rutils.APIKeyEnvVar))
	apiKeyID := firstNonEmpty(cc.cfg.APIKeyID, os.Getenv(rutils.APIKeyIDEnvVar))
	if partID == "" || apiKey == "" || apiKeyID == "" {
		cc.logger.Warn("sequence creation skipped: missing part_id/api_key/api_key_id (set via config or VIAM_* env)")
		return
	}

	ctx, cancel := context.WithTimeout(context.Background(), 20*time.Second)
	defer cancel()

	conn, err := rpc.DialDirectGRPC(ctx, "app.viam.com:443", cc.logger,
		rpc.WithEntityCredentials(apiKeyID, rpc.Credentials{
			Type:    rpc.CredentialsTypeAPIKey,
			Payload: apiKey,
		}))
	if err != nil {
		cc.logger.Warnf("sequence creation failed to dial app: %v", err)
		return
	}
	defer conn.Close()

	var resources []*datapb.SequenceResourceFilter
	for _, rm := range cc.cfg.resourceMethods() {
		resources = append(resources, &datapb.SequenceResourceFilter{
			ResourceName: rm.resource,
			MethodName:   rm.method,
		})
	}

	resp, err := datapb.NewDataServiceClient(conn).CreateSequence(ctx, &datapb.CreateSequenceRequest{
		PartId:       partID,
		Resources:    resources,
		SequenceTags: tags,
		StartTime:    timestamppb.New(start),
		EndTime:      timestamppb.New(end),
	})
	if err != nil {
		cc.logger.Warnf("sequence creation failed: %v", err)
		return
	}
	cc.logger.Infof("sequence created: id=%s tags=%v window=[%s, %s]",
		resp.GetId(), tags, start.Format(time.RFC3339), end.Format(time.RFC3339))
}

func (cc *captureControl) Close(ctx context.Context) error {
	cc.mu.Lock()
	defer cc.mu.Unlock()
	cc.capturing = false
	return nil
}
