package vive

import (
	"context"
	"fmt"
	"os"
	"path/filepath"
	"sync"
	"time"

	input "go.viam.com/rdk/components/input"
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

type ControllerConfig struct {
	SerialNumber string `json:"serial_number,omitempty"`
	DeviceName   string `json:"device_name,omitempty"`
}

func (cfg *ControllerConfig) Validate(path string) ([]string, []string, error) {
	return nil, nil, nil
}

type viveController struct {
	resource.AlwaysRebuild

	name   resource.Name
	logger logging.Logger
	cfg    *ControllerConfig

	cancelCtx  context.Context
	cancelFunc func()

	mu         sync.RWMutex
	deviceName string // resolved libsurvive object name (e.g. "WM0")
	lastState  *ControllerState
	lastEvents map[input.Control]input.Event
	callbacks  map[input.Control]map[input.EventType][]input.ControlFunction
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

// UpdateState reads the current controller state from libsurvive and updates events.
// Called by the teleop service's poll loop.
func (s *viveController) UpdateState() *ControllerState {
	s.mu.Lock()
	defer s.mu.Unlock()

	if s.deviceName == "" {
		return nil
	}

	data := survive.GetController(s.deviceName)
	if data == nil {
		s.lastState = nil
		return nil
	}

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
	}

	now := time.Now()

	// Update events for the standard input_controller API.
	s.updateEvent(input.AbsoluteX, cs.Trackpad[0], now)
	s.updateEvent(input.AbsoluteY, cs.Trackpad[1], now)
	s.updateEvent(input.AbsoluteZ, cs.Trigger, now)
	s.updateButtonEvent(input.ButtonSouth, cs.TrackpadPressed, now)
	s.updateButtonEvent(input.ButtonLT, cs.Grip, now)
	s.updateButtonEvent(input.ButtonMenu, cs.Menu, now)

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
