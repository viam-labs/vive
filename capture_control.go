package vive

import (
	"context"
	"fmt"
	"sync"
	"time"

	"go.viam.com/rdk/components/sensor"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/resource"
)

func init() {
	resource.RegisterComponent(sensor.API, CaptureControl,
		resource.Registration[sensor.Sensor, *CaptureControlConfig]{
			Constructor: newCaptureControl,
		},
	)
}

type CaptureControlConfig struct {
	ArmName            string   `json:"arm_name"`
	CameraNames        []string `json:"camera_names,omitempty"`
	CaptureFrequencyHz float64  `json:"capture_frequency_hz,omitempty"`
}

func (cfg *CaptureControlConfig) Validate(path string) ([]string, []string, error) {
	if cfg.ArmName == "" {
		return nil, nil, fmt.Errorf("%s: arm_name is required", path)
	}
	return nil, nil, nil
}

type captureControl struct {
	resource.AlwaysRebuild

	name   resource.Name
	logger logging.Logger
	cfg    *CaptureControlConfig

	mu            sync.RWMutex
	capturing     bool
	captureFreqHz float64
	task          string
	sessionTags   []string
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

	return &captureControl{
		name:          rawConf.ResourceName(),
		logger:        logger,
		cfg:           conf,
		captureFreqHz: freqHz,
	}, nil
}

func (cc *captureControl) Name() resource.Name {
	return cc.name
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

	overrides = append(overrides, map[string]interface{}{
		"resource_name":        cc.cfg.ArmName,
		"method":               "EndPosition",
		"capture_frequency_hz": freq,
		"tags":                 tags,
	})

	for _, camName := range cc.cfg.CameraNames {
		overrides = append(overrides, map[string]interface{}{
			"resource_name":        camName,
			"method":               "GetImages",
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

		cc.capturing = true
		sessionTag := fmt.Sprintf("session:%s", time.Now().Format("20060102_150405"))
		cc.sessionTags = []string{sessionTag}
		if cc.task != "" {
			cc.sessionTags = append(cc.sessionTags, fmt.Sprintf("cmd:%s", cc.task))
		}

		cc.logger.Infof("capture started: tags=%v freq=%.1fHz", cc.sessionTags, cc.captureFreqHz)
		return map[string]interface{}{
			"capturing": true,
			"tags":      cc.sessionTags,
		}, nil
	}

	if _, ok := cmd["stop-capture"]; ok {
		cc.mu.Lock()
		defer cc.mu.Unlock()

		cc.capturing = false
		cc.logger.Info("capture stopped")
		cc.sessionTags = nil

		return map[string]interface{}{
			"capturing": false,
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
			"tags":                 cc.sessionTags,
			"task":                 cc.task,
		}, nil
	}

	return nil, fmt.Errorf("unknown command: %v", cmd)
}

func (cc *captureControl) Close(ctx context.Context) error {
	cc.mu.Lock()
	defer cc.mu.Unlock()
	cc.capturing = false
	return nil
}
