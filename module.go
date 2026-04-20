package vive

import (
	"context"
	"fmt"
	"math"
	"os"
	"path/filepath"
	"sync"
	"time"

	input "go.viam.com/rdk/components/input"
	toggleswitch "go.viam.com/rdk/components/switch"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/resource"

	"vive/survive"
)

func init() {
	resource.RegisterComponent(input.API, ViveController,
		resource.Registration[input.Controller, *ControllerConfig]{
			Constructor: newViveController,
		},
	)
}

// ButtonAction defines a configurable button action, following the streamdeck pattern.
type ButtonAction struct {
	Component string        `json:"component"` // resource short name
	Method    string        `json:"method"`     // "do_command", "set_position", etc.
	Args      []interface{} `json:"args"`       // method-specific arguments
}

type ControllerConfig struct {
	SerialNumber        string        `json:"serial_number,omitempty"`
	DeviceName          string        `json:"device_name,omitempty"`
	MenuAction          *ButtonAction `json:"menu_action,omitempty"`
	TrackpadUpAction    *ButtonAction `json:"trackpad_up_action,omitempty"`
	TrackpadDownAction  *ButtonAction `json:"trackpad_down_action,omitempty"`
	TrackpadLeftAction  *ButtonAction `json:"trackpad_left_action,omitempty"`
	TrackpadRightAction *ButtonAction `json:"trackpad_right_action,omitempty"`
}

func (cfg *ControllerConfig) Validate(path string) ([]string, []string, error) {
	var deps []string
	seen := map[string]bool{}
	for _, action := range []*ButtonAction{
		cfg.MenuAction, cfg.TrackpadUpAction, cfg.TrackpadDownAction,
		cfg.TrackpadLeftAction, cfg.TrackpadRightAction,
	} {
		if action == nil || seen[action.Component] {
			continue
		}
		seen[action.Component] = true
		deps = append(deps, action.Component)
	}
	// Return deps as implicit_dependencies (second return value) to avoid
	// circular dependency errors. Implicit deps are resolved if available
	// but don't block construction if missing.
	return nil, deps, nil
}

type viveController struct {
	resource.AlwaysRebuild

	name   resource.Name
	logger logging.Logger
	cfg    *ControllerConfig
	deps   resource.Dependencies

	cancelCtx  context.Context
	cancelFunc func()

	mu                 sync.RWMutex
	deviceName         string // resolved libsurvive object name (e.g. "WM0")
	lastState          *ControllerState
	lastRawButtons     int32 // previous raw button mask for edge detection
	wasMenu            bool
	wasTrackpad        bool
	wasNil             bool // tracks nil→valid transitions for edge-state reset
	lastTimecode       float64
	lastTimecodeChange time.Time
	lastEvents         map[input.Control]input.Event
	callbacks          map[input.Control]map[input.EventType][]input.ControlFunction
}

// Available controls on a Vive Wand.
var viveControls = []input.Control{
	input.AbsoluteX,   // trackpad X
	input.AbsoluteY,   // trackpad Y
	input.AbsoluteZ,   // trigger (0-1)
	input.ButtonSouth, // trackpad press
	input.ButtonLT,    // grip
	input.ButtonMenu,  // menu
}

func newViveController(ctx context.Context, deps resource.Dependencies, rawConf resource.Config, logger logging.Logger) (input.Controller, error) {
	conf, err := resource.NativeConfig[*ControllerConfig](rawConf)
	if err != nil {
		return nil, err
	}
	return NewViveController(ctx, deps, rawConf.ResourceName(), conf, logger)
}

func NewViveController(ctx context.Context, deps resource.Dependencies, name resource.Name, conf *ControllerConfig, logger logging.Logger) (input.Controller, error) {
	// Determine plugin path for libsurvive.
	exePath, err := os.Executable()
	if err != nil {
		return nil, fmt.Errorf("cannot determine executable path: %w", err)
	}
	exeDir := filepath.Dir(exePath)
	pluginLib := filepath.Join(exeDir, "libsurvive", "lib", "libsurvive.so")
	if _, err := os.Stat(pluginLib); err != nil {
		pluginLib = filepath.Join(exeDir, "libsurvive", "lib", "libsurvive.dylib")
	}

	if err := survive.Acquire(pluginLib); err != nil {
		return nil, fmt.Errorf("failed to initialize libsurvive: %w", err)
	}

	cancelCtx, cancelFunc := context.WithCancel(context.Background())

	s := &viveController{
		name:       name,
		logger:     logger,
		cfg:        conf,
		deps:       deps,
		cancelCtx:  cancelCtx,
		cancelFunc: cancelFunc,
		deviceName: conf.DeviceName,
		lastEvents: make(map[input.Control]input.Event),
		callbacks:  make(map[input.Control]map[input.EventType][]input.ControlFunction),
	}

	return s, nil
}

func (s *viveController) Name() resource.Name {
	return s.name
}

// DeviceName returns the resolved libsurvive device name for this controller.
func (s *viveController) DeviceName() string {
	s.mu.RLock()
	defer s.mu.RUnlock()
	return s.deviceName
}

// SetDeviceName sets the resolved libsurvive device name.
func (s *viveController) SetDeviceName(name string) {
	s.mu.Lock()
	defer s.mu.Unlock()
	s.deviceName = name
}

// SerialNumber returns the configured serial number for this controller.
func (s *viveController) SerialNumber() string {
	return s.cfg.SerialNumber
}

// InjectDep adds a resource to the controller's deps map for button action
// resolution. Used by the teleop service to inject itself (avoiding a circular
// dependency in the resource graph).
func (s *viveController) InjectDep(name resource.Name, r resource.Resource) {
	s.deps[name] = r
}

// UpdateState reads the current controller state from libsurvive and updates events.
// Called by the teleop service's poll loop.
func (s *viveController) UpdateState() *ControllerState {
	s.mu.Lock()
	defer s.mu.Unlock()

	if s.deviceName == "" {
		s.wasNil = true
		return nil
	}

	data := survive.GetController(s.deviceName)
	if data == nil {
		s.wasNil = true
		s.lastState = nil
		return nil
	}

	// When recovering from nil, reset staleness timer to give a fresh 2s window.
	if s.wasNil {
		s.lastTimecodeChange = time.Now()
	}

	// Detect stale controller: timecode only advances when new sensor data arrives.
	if data.Timecode != s.lastTimecode {
		s.lastTimecode = data.Timecode
		s.lastTimecodeChange = time.Now()
	} else if s.lastTimecode > 0 && !s.lastTimecodeChange.IsZero() && time.Since(s.lastTimecodeChange) > 2*time.Second {
		if !s.wasNil {
			// TODO: remove verbose logging after confirming reconnect fix works
			s.logger.Warnf("%s: timecode stale for %.1fs (frozen at %.3f), treating as disconnected",
				s.deviceName, time.Since(s.lastTimecodeChange).Seconds(), s.lastTimecode)
		}
		s.wasNil = true
		s.lastState = nil
		return nil
	}

	// Reset edge-detection state when returning from nil to prevent false button events.
	if s.wasNil {
		// TODO: remove verbose logging after confirming reconnect fix works
		s.logger.Infof("%s: data flowing again (timecode=%.3f)", s.deviceName, data.Timecode)
		s.wasNil = false
		s.wasMenu = false
		s.wasTrackpad = false
		s.lastRawButtons = data.RawButtons
	}

	// Log button transitions.
	changed := data.RawButtons ^ s.lastRawButtons
	for bit, name := range survive.RawButtonNames {
		if changed&bit == 0 {
			continue
		}
		if data.RawButtons&bit != 0 {
			s.logger.Infof("%s: %s pressed", s.deviceName, name)
		} else {
			s.logger.Infof("%s: %s released", s.deviceName, name)
		}
	}
	s.lastRawButtons = data.RawButtons

	trigger := data.Axis1X
	cs := &ControllerState{
		Connected:       data.PoseValid,
		Trigger:         trigger,
		TriggerPressed:  trigger > 0.8,
		Grip:            data.Pressed&survive.ButtonGrip != 0,
		Trackpad:        [2]float64{data.Axis0X, -data.Axis0Y},
		TrackpadPressed: data.Pressed&survive.ButtonTrackpad != 0,
		Menu:            data.Pressed&survive.ButtonMenu != 0,
	}
	if data.PoseValid {
		m := Mat34ToMat4(data.Mat)
		cs.Mat = m
		cs.Pos = [3]float64{m.At(0, 3), m.At(1, 3), m.At(2, 3)}
		cs.Quat = data.Quat
	}

	now := time.Now()

	// Update events for the standard input_controller API.
	s.updateEvent(input.AbsoluteX, cs.Trackpad[0], now)
	s.updateEvent(input.AbsoluteY, cs.Trackpad[1], now)
	s.updateEvent(input.AbsoluteZ, cs.Trigger, now)
	s.updateButtonEvent(input.ButtonSouth, cs.TrackpadPressed, now)
	s.updateButtonEvent(input.ButtonLT, cs.Grip, now)
	s.updateButtonEvent(input.ButtonMenu, cs.Menu, now)

	// Configurable button actions: edge detection and dispatch.
	// Menu rising edge.
	if cs.Menu && !s.wasMenu {
		s.executeAction(s.cfg.MenuAction)
	}
	s.wasMenu = cs.Menu

	// Trackpad rising edge: 4-way d-pad dispatch.
	if cs.TrackpadPressed && !s.wasTrackpad {
		x := cs.Trackpad[0]
		y := cs.Trackpad[1]
		if math.Abs(y) >= math.Abs(x) {
			if y < -0.3 {
				s.executeAction(s.cfg.TrackpadUpAction)
			} else if y > 0.3 {
				s.executeAction(s.cfg.TrackpadDownAction)
			}
		} else {
			if x < -0.3 {
				s.executeAction(s.cfg.TrackpadLeftAction)
			} else if x > 0.3 {
				s.executeAction(s.cfg.TrackpadRightAction)
			}
		}
	}
	s.wasTrackpad = cs.TrackpadPressed

	s.lastState = cs
	return cs
}

func (s *viveController) updateEvent(ctrl input.Control, value float64, now time.Time) {
	ev := input.Event{
		Time:    now,
		Event:   input.PositionChangeAbs,
		Control: ctrl,
		Value:   value,
	}
	s.lastEvents[ctrl] = ev
	s.fireCallbacks(ctrl, ev)
}

func (s *viveController) updateButtonEvent(ctrl input.Control, pressed bool, now time.Time) {
	var evType input.EventType
	var value float64
	if pressed {
		evType = input.ButtonPress
		value = 1.0
	} else {
		evType = input.ButtonRelease
		value = 0.0
	}
	ev := input.Event{
		Time:    now,
		Event:   evType,
		Control: ctrl,
		Value:   value,
	}
	s.lastEvents[ctrl] = ev
	s.fireCallbacks(ctrl, ev)
}

func (s *viveController) fireCallbacks(ctrl input.Control, ev input.Event) {
	if ctrlCBs, ok := s.callbacks[ctrl]; ok {
		if fns, ok := ctrlCBs[ev.Event]; ok {
			for _, fn := range fns {
				fn(s.cancelCtx, ev)
			}
		}
		if fns, ok := ctrlCBs[input.AllEvents]; ok {
			for _, fn := range fns {
				fn(s.cancelCtx, ev)
			}
		}
	}
}

// HasTrackpadUpAction returns true if a trackpad up action is configured.
// The teleop service checks this to decide whether to run its internal calibrate.
func (s *viveController) HasTrackpadUpAction() bool {
	return s.cfg.TrackpadUpAction != nil
}

// executeAction resolves a button action's target component and dispatches by method.
func (s *viveController) executeAction(action *ButtonAction) {
	if action == nil {
		return
	}
	go func() {
		// Haptic feedback on button press (async to avoid blocking UpdateState).
		if s.deviceName != "" {
			survive.Haptic(s.deviceName, 0.3, 80.0/1000.0)
		}
		// Resolve resource by short name (like streamdeck's getResourceAndCommandForKey).
		var r resource.Resource
		if action.Component == s.name.ShortName() {
			r = s
		} else {
			for n, dep := range s.deps {
				if n.ShortName() == action.Component {
					r = dep
					break
				}
			}
		}
		if r == nil {
			s.logger.Warnf("button action: component %q not found in dependencies", action.Component)
			return
		}

		switch action.Method {
		case "do_command":
			cmd := map[string]interface{}{}
			if len(action.Args) > 0 {
				if m, ok := action.Args[0].(map[string]interface{}); ok {
					cmd = m
				}
			}
			if _, err := r.DoCommand(s.cancelCtx, cmd); err != nil {
				s.logger.Warnf("button action DoCommand on %q failed: %v", action.Component, err)
			}
		case "set_position":
			sw, ok := r.(toggleswitch.Switch)
			if !ok {
				s.logger.Warnf("button action: %q is not a switch component", action.Component)
				return
			}
			if len(action.Args) == 0 {
				s.logger.Warnf("button action: set_position on %q requires a position argument", action.Component)
				return
			}
			pos, ok := action.Args[0].(float64)
			if !ok {
				s.logger.Warnf("button action: set_position arg must be a number, got %T", action.Args[0])
				return
			}
			if err := sw.SetPosition(s.cancelCtx, uint32(pos), nil); err != nil {
				s.logger.Warnf("button action SetPosition on %q failed: %v", action.Component, err)
			}
		default:
			s.logger.Warnf("button action: unsupported method %q", action.Method)
		}
	}()
}

// Controls returns the list of controls provided by this controller.
func (s *viveController) Controls(ctx context.Context, extra map[string]interface{}) ([]input.Control, error) {
	return viveControls, nil
}

// Events returns the most recent event for each control.
func (s *viveController) Events(ctx context.Context, extra map[string]interface{}) (map[input.Control]input.Event, error) {
	s.mu.RLock()
	defer s.mu.RUnlock()
	result := make(map[input.Control]input.Event, len(s.lastEvents))
	for k, v := range s.lastEvents {
		result[k] = v
	}
	return result, nil
}

func (s *viveController) TriggerEvent(ctx context.Context, event input.Event, extra map[string]interface{}) error {
	return fmt.Errorf("TriggerEvent not supported on physical VR controllers")
}

func (s *viveController) RegisterControlCallback(ctx context.Context, control input.Control, triggers []input.EventType, ctrlFunc input.ControlFunction, extra map[string]interface{}) error {
	s.mu.Lock()
	defer s.mu.Unlock()
	if _, ok := s.callbacks[control]; !ok {
		s.callbacks[control] = make(map[input.EventType][]input.ControlFunction)
	}
	for _, t := range triggers {
		s.callbacks[control][t] = append(s.callbacks[control][t], ctrlFunc)
	}
	return nil
}

func (s *viveController) DoCommand(ctx context.Context, cmd map[string]interface{}) (map[string]interface{}, error) {
	if _, ok := cmd["get_pose"]; ok {
		s.mu.RLock()
		cs := s.lastState
		s.mu.RUnlock()
		if cs == nil || !cs.Connected {
			return map[string]interface{}{"valid": false}, nil
		}
		return map[string]interface{}{
			"valid":    true,
			"position": []float64{cs.Pos[0], cs.Pos[1], cs.Pos[2]},
			"matrix":   cs.Mat,
		}, nil
	}

	if hapticCmd, ok := cmd["haptic"]; ok {
		s.mu.RLock()
		devName := s.deviceName
		s.mu.RUnlock()
		if devName == "" {
			return nil, fmt.Errorf("controller not assigned to a device")
		}
		if hapticMap, ok := hapticCmd.(map[string]interface{}); ok {
			amp := 0.5
			dur := 100.0
			if v, ok := hapticMap["amplitude"]; ok {
				if f, ok := v.(float64); ok {
					amp = f
				}
			}
			if v, ok := hapticMap["duration_ms"]; ok {
				if f, ok := v.(float64); ok {
					dur = f
				}
			}
			survive.Haptic(devName, amp, dur)
		}
		return map[string]interface{}{"ok": true}, nil
	}

	if _, ok := cmd["status"]; ok {
		s.mu.RLock()
		devName := s.deviceName
		cs := s.lastState
		s.mu.RUnlock()
		connected := cs != nil && cs.Connected
		poseValid := cs != nil && cs.Connected
		return map[string]interface{}{
			"device_name": devName,
			"serial":      s.cfg.SerialNumber,
			"connected":   connected,
			"pose_valid":  poseValid,
		}, nil
	}

	return nil, fmt.Errorf("unknown command: %v", cmd)
}

func (s *viveController) Close(context.Context) error {
	s.cancelFunc()
	survive.Release()
	return nil
}
