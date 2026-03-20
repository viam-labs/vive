package vive

import (
	"context"
	"encoding/json"
	"fmt"
	"math"
	"os"
	"os/exec"
	"path/filepath"
	"strings"
	"sync"
	"sync/atomic"
	"time"

	"github.com/go-gl/mathgl/mgl64"
	"github.com/golang/geo/r3"
	"go.viam.com/rdk/components/arm"
	"go.viam.com/rdk/components/gripper"
	input "go.viam.com/rdk/components/input"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/resource"
	"go.viam.com/rdk/services/generic"
	"go.viam.com/rdk/services/motion"
	"go.viam.com/rdk/spatialmath"

	"vive/survive"
)

func init() {
	resource.RegisterService(generic.API, ViveTeleop,
		resource.Registration[resource.Resource, *TeleopConfig]{
			Constructor: newTeleopService,
		},
	)
}

type HandConfig struct {
	Name            string  `json:"name"`
	Controller      string  `json:"controller"`
	Arm             string  `json:"arm"`
	Gripper         string  `json:"gripper,omitempty"`
	Scale           float64 `json:"scale,omitempty"`
	RotationEnabled *bool   `json:"rotation_enabled,omitempty"`
	PosDeadzoneMM   float64 `json:"pos_deadzone_mm,omitempty"`
	RotDeadzoneDeg  float64 `json:"rot_deadzone_deg,omitempty"`
	SmoothAlpha     float64 `json:"smooth_alpha,omitempty"`
}

type TeleopConfig struct {
	Hz             int          `json:"hz,omitempty"`
	Hands          []HandConfig `json:"hands"`
	CalibrationDir string       `json:"calibration_dir,omitempty"`
}

func (cfg *TeleopConfig) Validate(path string) ([]string, []string, error) {
	if len(cfg.Hands) == 0 {
		return nil, nil, fmt.Errorf("%s: at least one hand must be configured", path)
	}
	var deps []string
	for _, h := range cfg.Hands {
		if h.Controller == "" {
			return nil, nil, fmt.Errorf("%s: hand %q must have a controller", path, h.Name)
		}
		if h.Arm == "" {
			return nil, nil, fmt.Errorf("%s: hand %q must have an arm", path, h.Name)
		}
		deps = append(deps, h.Controller, h.Arm)
		if h.Gripper != "" {
			deps = append(deps, h.Gripper)
		}
	}
	// Motion service dependency.
	deps = append(deps, "rdk:service:motion/builtin")
	return deps, nil, nil
}

type teleopService struct {
	resource.AlwaysRebuild

	name   resource.Name
	logger logging.Logger
	cfg    *TeleopConfig

	cancelCtx  context.Context
	cancelFunc func()

	hands []*teleopHand

	// controller discovery state
	controllersAssigned bool
	firstControllerSeen time.Time
	serialWaitLogged    bool

	// calibration state (shared across all hands)
	calibMu  sync.RWMutex
	calibYaw float64
	calibSet bool
	calibDir string

	// frameChecked tracks whether pollLoop has verified base station frame orientation.
	frameChecked atomic.Bool

	// surviveMu guards libsurvive access during recalibration/pairing to prevent
	// the poll loop from calling into a destroyed context.
	surviveMu sync.Mutex
}

type teleopHand struct {
	name         string
	controller   *viveController
	arm          arm.Arm
	gripper      gripper.Gripper
	motionSvc    motion.Service
	armName      string
	gripperName  string
	scale        float64
	rotEnabled   bool
	firstDataLog bool // true after first successful tracking data logged
	absoluteRot  bool
	armFrameMat  mgl64.Mat4

	// button edge state
	wasGrip     bool
	wasMenu     bool
	wasTrackpad bool

	// gripper proportional control
	gripperDesired atomic.Int64
	gripperNotify  chan struct{}

	// control state
	isControlling bool
	teleopActive  bool
	errorTimeout  time.Time
	lastCmdTime   time.Time
	movePending   atomic.Bool
	poseStack     []Pose

	// session logging
	logFile *os.File

	// dead-zone filtering
	posDeadzone      float64
	rotDeadzone      float64
	lastSentPose     *Pose
	deadzoneFiltered int

	// EMA smoothing state
	smoothAlpha  float64
	smoothedPose *Pose

	// reference capture (on grip press)
	ctrlRefPos      [3]float64
	ctrlRefRotRobot mgl64.Mat4
	calibTransform  mgl64.Mat4
	robotRefPos     [3]float64
	robotRefMat     mgl64.Mat4
	ctrlToArmOffset mgl64.Mat4

	// parent service for calibration access
	svc *teleopService
}

const (
	cmdInterval   = 1 * time.Millisecond
	errorCooldown = 1 * time.Second
)

func newTeleopService(ctx context.Context, deps resource.Dependencies, rawConf resource.Config, logger logging.Logger) (resource.Resource, error) {
	conf, err := resource.NativeConfig[*TeleopConfig](rawConf)
	if err != nil {
		return nil, err
	}
	return NewTeleopService(ctx, deps, rawConf.ResourceName(), conf, logger)

}

func NewTeleopService(ctx context.Context, deps resource.Dependencies, name resource.Name, conf *TeleopConfig, logger logging.Logger) (resource.Resource, error) {
	hz := conf.Hz
	if hz <= 0 {
		hz = 90
	}

	calibDir := conf.CalibrationDir
	if calibDir == "" {
		exePath, err := os.Executable()
		if err != nil {
			return nil, fmt.Errorf("cannot determine executable path: %w", err)
		}
		calibDir = filepath.Dir(exePath)
	}

	cancelCtx, cancelFunc := context.WithCancel(context.Background())

	svc := &teleopService{
		name:       name,
		logger:     logger,
		cfg:        conf,
		cancelCtx:  cancelCtx,
		cancelFunc: cancelFunc,
		calibDir:   calibDir,
	}

	// Load calibration.
	svc.loadCalib()

	// Resolve dependencies and create hands.
	motionSvc, err := motion.FromDependencies(deps, "builtin")
	if err != nil {
		logger.Warnf("motion service not available: %v", err)
	}

	for _, hc := range conf.Hands {
		ctrl, err := input.FromDependencies(deps, hc.Controller)
		if err != nil {
			return nil, fmt.Errorf("controller %q: %w", hc.Controller, err)
		}
		vc, ok := ctrl.(*viveController)
		if !ok {
			return nil, fmt.Errorf("controller %q is not a vive controller", hc.Controller)
		}

		a, err := arm.FromDependencies(deps, hc.Arm)
		if err != nil {
			return nil, fmt.Errorf("arm %q: %w", hc.Arm, err)
		}

		var g gripper.Gripper
		if hc.Gripper != "" {
			g, err = gripper.FromDependencies(deps, hc.Gripper)
			if err != nil {
				logger.Warnf("gripper %q not available: %v", hc.Gripper, err)
			}
		}

		scale := hc.Scale
		if scale <= 0 {
			scale = 1.0
		}
		rotEnabled := true
		if hc.RotationEnabled != nil {
			rotEnabled = *hc.RotationEnabled
		}
		posDeadzone := hc.PosDeadzoneMM
		if posDeadzone <= 0 {
			posDeadzone = 2.0
		}
		rotDeadzone := hc.RotDeadzoneDeg
		if rotDeadzone <= 0 {
			rotDeadzone = 2.0
		}
		smoothAlpha := hc.SmoothAlpha
		if smoothAlpha <= 0 {
			smoothAlpha = 0.2
		}

		h := &teleopHand{
			name:            hc.Name,
			controller:      vc,
			arm:             a,
			gripper:         g,
			motionSvc:       motionSvc,
			armName:         hc.Arm,
			gripperName:     hc.Gripper,
			scale:           scale,
			rotEnabled:      rotEnabled,
			absoluteRot:     true,
			armFrameMat:     mgl64.Ident4(),
			ctrlToArmOffset: mgl64.Ident4(),
			gripperNotify:   make(chan struct{}, 1),
			posDeadzone:     posDeadzone,
			rotDeadzone:     rotDeadzone,
			smoothAlpha:     smoothAlpha,
			svc:             svc,
		}
		h.gripperDesired.Store(830)

		if h.gripper != nil {
			go h.gripperLoop(cancelCtx)
		}
		if h.motionSvc != nil {
			go h.teleopStatusLoop(cancelCtx)
		}

		svc.hands = append(svc.hands, h)
	}

	// Start poll loop.
	go svc.pollLoop(cancelCtx, hz)

	return svc, nil
}

func (svc *teleopService) Name() resource.Name {
	return svc.name
}

func (svc *teleopService) DoCommand(ctx context.Context, cmd map[string]interface{}) (map[string]interface{}, error) {
	if _, ok := cmd["calibrate"]; ok {
		// Use the first hand's controller for calibration.
		for _, h := range svc.hands {
			cs := h.controller.UpdateState()
			if cs != nil && cs.Connected {
				if yaw, ok := ComputeCalibYaw(cs.Mat); ok {
					svc.saveCalib(yaw)
					return map[string]interface{}{"calibrated": true, "yaw_deg": yaw * 180 / math.Pi}, nil
				}
			}
		}
		return nil, fmt.Errorf("no connected controller available for calibration")
	}

	if _, ok := cmd["status"]; ok {
		status := make([]map[string]interface{}, len(svc.hands))
		for i, h := range svc.hands {
			status[i] = map[string]interface{}{
				"name":          h.name,
				"controlling":   h.isControlling,
				"teleop_active": h.teleopActive,
			}
		}
		return map[string]interface{}{"hands": status}, nil
	}

	if _, ok := cmd["toggle_rotation_mode"]; ok {
		for _, h := range svc.hands {
			h.absoluteRot = !h.absoluteRot
		}
		mode := "relative"
		if len(svc.hands) > 0 && svc.hands[0].absoluteRot {
			mode = "absolute"
		}
		return map[string]interface{}{"rotation_mode": mode}, nil
	}

	if _, ok := cmd["list_controllers"]; ok {
		count := survive.ObjectCount()
		var result []map[string]interface{}
		for i := 0; i < count; i++ {
			info := survive.GetObjectInfo(i)
			if !info.IsController {
				continue
			}
			assignedTo := ""
			for _, h := range svc.hands {
				if h.controller.DeviceName() == info.Name {
					assignedTo = h.name
					break
				}
			}
			result = append(result, map[string]interface{}{
				"name":        info.Name,
				"serial":      info.Serial,
				"assigned_to": assignedTo,
			})
		}
		return map[string]interface{}{"controllers": result}, nil
	}

	if assignCmd, ok := cmd["assign"]; ok {
		assignMap, ok := assignCmd.(map[string]interface{})
		if !ok {
			return nil, fmt.Errorf("assign: expected {\"serial\": \"...\", \"hand\": \"...\"}")
		}
		serial, _ := assignMap["serial"].(string)
		handName, _ := assignMap["hand"].(string)
		if serial == "" || handName == "" {
			return nil, fmt.Errorf("assign: serial and hand are required")
		}

		// Find the hand.
		var targetHand *teleopHand
		for _, h := range svc.hands {
			if h.name == handName {
				targetHand = h
				break
			}
		}
		if targetHand == nil {
			return nil, fmt.Errorf("assign: unknown hand %q", handName)
		}

		// Find the device name for this serial.
		count := survive.ObjectCount()
		deviceName := ""
		for i := 0; i < count; i++ {
			info := survive.GetObjectInfo(i)
			if info.Serial == serial {
				deviceName = info.Name
				break
			}
		}
		if deviceName == "" {
			return nil, fmt.Errorf("assign: no controller with serial %q found", serial)
		}

		targetHand.controller.SetDeviceName(deviceName)

		// Update and save the controller map.
		cm := loadControllerMap(svc.calibDir)
		for i, h := range svc.hands {
			if h == targetHand {
				if i == 0 {
					cm.Left = serial
				} else if i == 1 {
					cm.Right = serial
				}
			}
		}
		saveControllerMap(svc.calibDir, cm, svc.logger)
		svc.logger.Infof("assigned serial %s to hand %q (device=%s)", serial, handName, deviceName)
		return map[string]interface{}{"assigned": true, "device": deviceName}, nil
	}

	if _, ok := cmd["pair_mode"]; ok {
		exePath, err := os.Executable()
		if err != nil {
			return nil, fmt.Errorf("pair_mode: cannot determine executable path: %w", err)
		}
		surviveCLI := filepath.Join(filepath.Dir(exePath), "libsurvive", "bin", "survive-cli")
		if _, err := os.Stat(surviveCLI); err != nil {
			return nil, fmt.Errorf("pair_mode: survive-cli not found at %s", surviveCLI)
		}

		svc.logger.Info("Starting dongle pairing mode...")

		// Lock out the poll loop while libsurvive is released.
		svc.surviveMu.Lock()

		// Release libsurvive so survive-cli can take over USB devices.
		for range svc.hands {
			survive.Release()
		}

		pairCtx, pairCancel := context.WithTimeout(ctx, 120*time.Second)
		defer pairCancel()

		pairCmd := exec.CommandContext(pairCtx, surviveCLI, "--pair-device")
		pairCmd.Stdout = os.Stdout
		pairCmd.Stderr = os.Stderr
		err = pairCmd.Run()

		// Re-acquire libsurvive.
		pluginLib := filepath.Join(filepath.Dir(exePath), "libsurvive", "lib", "libsurvive.so")
		if _, statErr := os.Stat(pluginLib); statErr != nil {
			pluginLib = filepath.Join(filepath.Dir(exePath), "libsurvive", "lib", "libsurvive.dylib")
		}
		for range svc.hands {
			_ = survive.Acquire(pluginLib)
		}
		svc.surviveMu.Unlock()
		svc.controllersAssigned = false

		if err != nil {
			return nil, fmt.Errorf("pair_mode: %w", err)
		}
		svc.logger.Info("Pairing complete")
		return map[string]interface{}{"paired": true}, nil
	}

	if _, ok := cmd["recalibrate"]; ok {
		svc.logger.Info("Recalibrating: clearing libsurvive base station cache and restarting tracking...")

		// Stop any active teleop hands.
		for _, h := range svc.hands {
			if h.teleopActive {
				h.stopTeleop(ctx)
			}
		}

		// Lock out the poll loop while we tear down and reinit libsurvive.
		svc.surviveMu.Lock()

		// Release libsurvive (same pattern as pair_mode).
		for range svc.hands {
			survive.Release()
		}

		// Delete libsurvive cached base station config.
		homeDir, _ := os.UserHomeDir()
		lsConfig := filepath.Join(homeDir, ".config", "libsurvive", "config.json")
		if err := os.Remove(lsConfig); err != nil && !os.IsNotExist(err) {
			svc.logger.Warnf("Failed to remove libsurvive config: %v", err)
		}

		// Clear yaw calibration.
		svc.calibMu.Lock()
		svc.calibYaw = 0
		svc.calibSet = false
		svc.calibMu.Unlock()
		calibPath := filepath.Join(svc.calibDir, "calibration.json")
		os.Remove(calibPath)

		// Re-acquire libsurvive (re-solves base station geometry).
		exePath, _ := os.Executable()
		pluginLib := filepath.Join(filepath.Dir(exePath), "libsurvive", "lib", "libsurvive.so")
		if _, statErr := os.Stat(pluginLib); statErr != nil {
			pluginLib = filepath.Join(filepath.Dir(exePath), "libsurvive", "lib", "libsurvive.dylib")
		}
		for range svc.hands {
			_ = survive.Acquire(pluginLib)
		}

		svc.surviveMu.Unlock()

		// Reset discovery and frame-check state so poll loop re-discovers controllers and re-checks frame.
		svc.controllersAssigned = false
		svc.frameChecked.Store(false)

		svc.logger.Info("Recalibration complete — base stations will re-solve. Use 'calibrate' or trackpad-up to set forward direction.")
		return map[string]interface{}{"recalibrated": true}, nil
	}

	return nil, fmt.Errorf("unknown command: %v", cmd)
}

func (svc *teleopService) Close(ctx context.Context) error {
	svc.cancelFunc()
	for _, h := range svc.hands {
		if h.teleopActive {
			h.stopTeleop(ctx)
		}
	}
	return nil
}

// ---------------------------------------------------------------------------
// Calibration
// ---------------------------------------------------------------------------

func ComputeCalibYaw(m mgl64.Mat4) (float64, bool) {
	cfx := m.At(0, 1)
	cfy := m.At(1, 1)
	mag := math.Sqrt(cfx*cfx + cfy*cfy)
	if mag < 1e-6 {
		return 0, false
	}
	return math.Atan2(-cfx/mag, cfy/mag), true
}

type calibData struct {
	Yaw float64 `json:"yaw"`
}

func (svc *teleopService) loadCalib() {
	path := filepath.Join(svc.calibDir, "calibration.json")
	b, err := os.ReadFile(path)
	if err != nil {
		svc.logger.Info("No calibration file, forward = raw tracking frame")
		return
	}
	var cd calibData
	if err := json.Unmarshal(b, &cd); err != nil {
		svc.logger.Warnf("Bad calibration file: %v", err)
		return
	}
	svc.calibMu.Lock()
	svc.calibYaw = cd.Yaw
	svc.calibSet = true
	svc.calibMu.Unlock()
	svc.logger.Infof("Calibration loaded: %.1f°", cd.Yaw*180/math.Pi)
}

func (svc *teleopService) saveCalib(yaw float64) {
	svc.calibMu.Lock()
	svc.calibYaw = yaw
	svc.calibSet = true
	svc.calibMu.Unlock()
	path := filepath.Join(svc.calibDir, "calibration.json")
	b, _ := json.Marshal(calibData{Yaw: yaw})
	if err := os.WriteFile(path, b, 0644); err != nil {
		svc.logger.Warnf("Failed to save calibration: %v", err)
	} else {
		svc.logger.Infof("Calibrated forward: %.1f° (saved)", yaw*180/math.Pi)
	}
}

// ---------------------------------------------------------------------------
// Controller discovery
// ---------------------------------------------------------------------------

type controllerMapData struct {
	Left  string `json:"left"`
	Right string `json:"right"`
}

func loadControllerMap(dir string) controllerMapData {
	var m controllerMapData
	b, err := os.ReadFile(filepath.Join(dir, "controller_map.json"))
	if err != nil {
		return m
	}
	json.Unmarshal(b, &m)
	return m
}

func saveControllerMap(dir string, m controllerMapData, logger logging.Logger) {
	b, _ := json.MarshalIndent(m, "", "  ")
	if err := os.WriteFile(filepath.Join(dir, "controller_map.json"), b, 0644); err != nil {
		logger.Warnf("failed to save controller map: %v", err)
	}
}

// discoverAndAssignControllers finds libsurvive objects and assigns them to
// the viveController instances based on serial numbers from controller_map.json.
func (svc *teleopService) discoverAndAssignControllers() {
	count := survive.ObjectCount()
	if count == 0 {
		return
	}
	type discovered struct {
		name   string
		serial string
	}
	var controllers []discovered
	for i := 0; i < count; i++ {
		info := survive.GetObjectInfo(i)
		if info.IsController {
			controllers = append(controllers, discovered{name: info.Name, serial: info.Serial})
		}
	}
	if len(controllers) == 0 {
		if !svc.controllersAssigned {
			svc.logger.Debugf("found %d tracked objects but 0 controllers", count)
		}
		return
	}

	if !svc.controllersAssigned {
		// Wait for serials if not all available yet (up to 15s after first discovery).
		if svc.firstControllerSeen.IsZero() {
			svc.firstControllerSeen = time.Now()
			svc.logger.Infof("found %d controllers, checking serial numbers...", len(controllers))
		}
		allSerialsAvailable := true
		for _, c := range controllers {
			if c.serial == "" {
				allSerialsAvailable = false
				break
			}
		}
		if !allSerialsAvailable && time.Since(svc.firstControllerSeen) < 15*time.Second {
			if !svc.serialWaitLogged {
				svc.logger.Info("Waiting for controller serial numbers (up to 15s)...")
				svc.serialWaitLogged = true
			}
			return
		}

		for _, c := range controllers {
			svc.logger.Infof("discovered controller %s serial=%s", c.name, c.serial)
		}
	}

	// Build serial→name lookup.
	serialToName := map[string]string{}
	for _, c := range controllers {
		if c.serial != "" {
			serialToName[c.serial] = c.name
		}
	}

	// Load saved serial mapping.
	cm := loadControllerMap(svc.calibDir)

	// For each hand, try to match its controller by serial.
	// Hands are ordered; first hand gets first controller if no serial match.
	assigned := map[string]bool{}
	for _, h := range svc.hands {
		serial := h.controller.SerialNumber()
		if serial != "" {
			if name, ok := serialToName[serial]; ok {
				h.controller.SetDeviceName(name)
				assigned[name] = true
				continue
			}
		}
	}

	// Try saved map: first hand = "left", second hand = "right" by convention.
	for i, h := range svc.hands {
		if h.controller.DeviceName() != "" {
			continue
		}
		var savedSerial string
		if i == 0 {
			savedSerial = cm.Left
		} else if i == 1 {
			savedSerial = cm.Right
		}
		if savedSerial != "" {
			if name, ok := serialToName[savedSerial]; ok && !assigned[name] {
				h.controller.SetDeviceName(name)
				assigned[name] = true
				continue
			}
		}
	}

	// Assign remaining controllers to unassigned hands in order.
	for _, h := range svc.hands {
		if h.controller.DeviceName() != "" {
			continue
		}
		for _, c := range controllers {
			if !assigned[c.name] {
				h.controller.SetDeviceName(c.name)
				assigned[c.name] = true
				break
			}
		}
	}

	// Update and save the controller map.
	needSave := false
	for i, h := range svc.hands {
		devName := h.controller.DeviceName()
		if devName == "" {
			continue
		}
		for _, c := range controllers {
			if c.name == devName && c.serial != "" {
				if i == 0 && cm.Left != c.serial {
					cm.Left = c.serial
					needSave = true
				} else if i == 1 && cm.Right != c.serial {
					cm.Right = c.serial
					needSave = true
				}
			}
		}
	}
	if needSave {
		saveControllerMap(svc.calibDir, cm, svc.logger)
		svc.logger.Infof("controller map saved: left=%s right=%s", cm.Left, cm.Right)
	} else if !svc.controllersAssigned && (cm.Left != "" || cm.Right != "") {
		svc.logger.Infof("controller map matched: left=%s right=%s", cm.Left, cm.Right)
	}

	if !svc.controllersAssigned {
		for _, h := range svc.hands {
			svc.logger.Infof("hand %q → device %q", h.name, h.controller.DeviceName())
		}
		svc.controllersAssigned = true
	}
}

// ---------------------------------------------------------------------------
// Poll loop
// ---------------------------------------------------------------------------

func (svc *teleopService) pollLoop(ctx context.Context, hz int) {
	interval := time.Duration(float64(time.Second) / float64(hz))
	svc.logger.Infof("Polling at %d Hz (%.1f ms)", hz, float64(interval)/float64(time.Millisecond))

	lastScan := time.Time{}

	for {
		start := time.Now()

		// Skip this iteration if libsurvive is being restarted (recalibrate/pair).
		if !svc.surviveMu.TryLock() {
			time.Sleep(interval)
			continue
		}

		// Periodic device scan (every 2s).
		if time.Since(lastScan) > 2*time.Second {
			// Check frame orientation once after libsurvive solves base station positions.
			if !svc.frameChecked.Load() {
				upZ := survive.FrameUpZ()
				if upZ != 0 {
					svc.frameChecked.Store(true)
					if upZ < 0 {
						svc.logger.Warnf("Frame Z-flip detected (upZ=%.2f), applying 180° X correction", upZ)
						LighthouseTransform = mgl64.HomogRotate3DX(math.Pi).Mul4(LighthouseTransform)
					} else {
						svc.logger.Infof("Frame orientation OK (upZ=%.2f)", upZ)
					}
				}
			}
			svc.discoverAndAssignControllers()
			lastScan = time.Now()
		}

		survive.PollEvents()
		svc.surviveMu.Unlock()

		for _, h := range svc.hands {
			cs := h.controller.UpdateState()
			if cs == nil {
				continue
			}
			if !h.firstDataLog {
				h.firstDataLog = true
				svc.logger.Infof("[%s] receiving tracking data (device=%s, connected=%v)", h.name, h.controller.DeviceName(), cs.Connected)
			}
			h.tick(ctx, *cs)
		}

		elapsed := time.Since(start)
		if sleep := interval - elapsed; sleep > 0 {
			select {
			case <-ctx.Done():
				return
			case <-time.After(sleep):
			}
		} else {
			select {
			case <-ctx.Done():
				return
			default:
			}
		}
	}
}

// ---------------------------------------------------------------------------
// Hand logic
// ---------------------------------------------------------------------------

func (h *teleopHand) teleopComponentName() string {
	if h.gripperName != "" {
		return h.gripperName
	}
	return h.armName
}

func (h *teleopHand) sendHaptic(intensity, durationMs float64) {
	devName := h.controller.DeviceName()
	if devName != "" {
		survive.Haptic(devName, intensity, durationMs)
	}
}

func (h *teleopHand) teleopStatusLoop(ctx context.Context) {
	ticker := time.NewTicker(200 * time.Millisecond)
	defer ticker.Stop()
	for {
		select {
		case <-ctx.Done():
			return
		case <-ticker.C:
		}
		if !h.teleopActive {
			continue
		}
		resp, err := h.motionSvc.DoCommand(ctx, map[string]interface{}{
			"teleop_status": true,
		})
		if err != nil {
			h.svc.logger.Warnf("[%s] teleop_status: %v", h.name, err)
			continue
		}
		if statusVal, ok := resp["teleop_status"]; ok {
			if statusMap, ok := statusVal.(map[string]interface{}); ok {
				if errVal, ok := statusMap["error"]; ok && errVal != nil {
					if errStr, ok := errVal.(string); ok && errStr != "" {
						h.svc.logger.Warnf("[%s] teleop error: %s", h.name, errStr)
						h.sendHaptic(0.8, 200)
					}
				}
			}
		}
	}
}

func (h *teleopHand) gripperLoop(ctx context.Context) {
	lastSent := int64(-1)
	for {
		select {
		case <-ctx.Done():
			return
		case <-h.gripperNotify:
		}
		pos := h.gripperDesired.Load()
		if pos == lastSent {
			continue
		}
		lastSent = pos
		cmd := map[string]interface{}{"set": int(pos)}
		if _, err := h.gripper.DoCommand(ctx, cmd); err != nil {
			h.svc.logger.Warnf("[%s] gripper set %d: %v", h.name, int(pos), err)
		}
	}
}

func (h *teleopHand) tick(ctx context.Context, cs ControllerState) {
	// Gripper: trigger → proportional position.
	if h.gripper != nil {
		pos := 830 - int(cs.Trigger*820)
		if pos < 10 {
			pos = 10
		}
		h.gripperDesired.Store(int64(pos))
		select {
		case h.gripperNotify <- struct{}{}:
		default:
		}
	}

	if !cs.Connected {
		return
	}

	// Trackpad rising edge: dispatch by region.
	if cs.TrackpadPressed && !h.wasTrackpad {
		y := cs.Trackpad[1]
		if y < -0.3 {
			// Up: calibrate forward direction.
			if yaw, ok := ComputeCalibYaw(cs.Mat); ok {
				h.svc.saveCalib(yaw)
				h.sendHaptic(0.3, 80)
			}
		} else if y > 0.3 {
			// Down: toggle rotation control mode.
			h.absoluteRot = !h.absoluteRot
			mode := "relative"
			if h.absoluteRot {
				mode = "absolute"
			}
			h.svc.logger.Infof("[%s] rotation mode: %s", h.name, mode)
			h.sendHaptic(0.3, 80)
		}
	}
	h.wasTrackpad = cs.TrackpadPressed

	// Grip: start/stop arm control.
	if cs.Grip && !h.wasGrip {
		h.startControl(ctx, cs)
	} else if !cs.Grip && h.wasGrip && h.isControlling {
		h.isControlling = false
		h.sendHaptic(0.3, 80)
		go h.stopTeleop(ctx)
	}
	h.wasGrip = cs.Grip

	// Menu: return to saved pose.
	if cs.Menu && !h.wasMenu && len(h.poseStack) > 0 {
		go h.returnToPose(ctx)
	}
	h.wasMenu = cs.Menu

	if h.isControlling {
		h.controlFrame(ctx, cs)
	}
}

func (h *teleopHand) startControl(ctx context.Context, cs ControllerState) {
	go func() {
		componentName := h.gripperName
		if componentName == "" {
			componentName = h.armName
		}
		var currentPose spatialmath.Pose
		if h.motionSvc != nil {
			pif, err := h.motionSvc.GetPose(ctx, componentName, "world", nil, nil)
			if err != nil {
				h.svc.logger.Warnf("[%s] startControl: GetPose: %v (falling back to arm.EndPosition)", h.name, err)
				ep, err2 := h.arm.EndPosition(ctx, nil)
				if err2 != nil {
					h.svc.logger.Warnf("[%s] startControl: EndPosition: %v", h.name, err2)
					return
				}
				currentPose = ep
			} else {
				currentPose = pif.Pose()
			}
		} else {
			ep, err := h.arm.EndPosition(ctx, nil)
			if err != nil {
				h.svc.logger.Warnf("[%s] startControl: EndPosition: %v", h.name, err)
				return
			}
			currentPose = ep
		}

		pt := currentPose.Point()
		ori := currentPose.Orientation()
		ovd := ori.OrientationVectorDegrees()

		h.robotRefPos = [3]float64{pt.X, pt.Y, pt.Z}

		ovRad := spatialmath.NewOrientationVector()
		ovRad.OX = ovd.OX
		ovRad.OY = ovd.OY
		ovRad.OZ = ovd.OZ
		ovRad.Theta = ovd.Theta * math.Pi / 180
		rq := ovRad.Quaternion()
		h.robotRefMat = mgl64.Quat{W: rq.Real, V: mgl64.Vec3{rq.Imag, rq.Jmag, rq.Kmag}}.Normalize().Mat4()

		h.poseStack = append(h.poseStack, Pose{
			X: pt.X, Y: pt.Y, Z: pt.Z,
			OX: ovd.OX, OY: ovd.OY, OZ: ovd.OZ, ThetaDeg: ovd.Theta,
		})

		h.ctrlRefPos = cs.Pos

		h.svc.calibMu.RLock()
		yaw := h.svc.calibYaw
		h.svc.calibMu.RUnlock()
		h.calibTransform = LighthouseTransform
		if yaw != 0 {
			yawM := mgl64.HomogRotate3DZ(yaw)
			h.calibTransform = LighthouseTransform.Mul4(yawM)
		}

		calibInv := h.calibTransform.Inv()
		h.ctrlRefRotRobot = h.calibTransform.Mul4(cs.Mat).Mul4(calibInv)
		h.ctrlToArmOffset = h.ctrlRefRotRobot.Inv().Mul4(h.robotRefMat)

		if h.motionSvc != nil {
			compName := h.teleopComponentName()
			startReq := fmt.Sprintf(
				`{"name":"builtin","component_name":"%s","destination":{"reference_frame":"world","pose":{"x":%f,"y":%f,"z":%f,"o_x":%f,"o_y":%f,"o_z":%f,"theta":%f}}}`,
				compName, pt.X, pt.Y, pt.Z, ovd.OX, ovd.OY, ovd.OZ, ovd.Theta,
			)
			if _, err := h.motionSvc.DoCommand(ctx, map[string]interface{}{
				"teleop_start": startReq,
			}); err != nil {
				h.svc.logger.Warnf("[%s] teleop_start failed: %v", h.name, err)
				h.sendHaptic(0.8, 200)
				return
			}
			h.teleopActive = true
			h.svc.logger.Infof("[%s] teleop_start sent for %s", h.name, compName)

			logName := fmt.Sprintf("teleop_%s_%s.jsonl", h.name, time.Now().Format("20060102_150405"))
			if f, err := os.Create(logName); err != nil {
				h.svc.logger.Warnf("[%s] failed to create log file %s: %v", h.name, logName, err)
			} else {
				h.logFile = f
			}
		}

		h.errorTimeout = time.Time{}
		h.lastSentPose = nil
		h.smoothedPose = nil
		h.isControlling = true
		h.sendHaptic(0.5, 100)
		h.svc.logger.Infof("[%s] control started at (%.1f, %.1f, %.1f)", h.name, pt.X, pt.Y, pt.Z)
	}()
}

func (h *teleopHand) controlFrame(ctx context.Context, cs ControllerState) {
	now := time.Now()
	if now.Sub(h.lastCmdTime) < cmdInterval {
		return
	}
	if !h.errorTimeout.IsZero() && now.Before(h.errorTimeout) {
		return
	}

	dx := cs.Pos[0] - h.ctrlRefPos[0]
	dy := cs.Pos[1] - h.ctrlRefPos[1]
	dz := cs.Pos[2] - h.ctrlRefPos[2]
	delta := h.calibTransform.Mul4x1(mgl64.Vec4{dx, dy, dz, 0})
	scaleMM := h.scale * 1000
	tx := h.robotRefPos[0] + delta[0]*scaleMM
	ty := h.robotRefPos[1] + delta[1]*scaleMM
	tz := h.robotRefPos[2] + delta[2]*scaleMM

	var tox, toy, toz, thetaDeg float64
	if h.rotEnabled {
		if h.absoluteRot {
			absRotRobot := h.calibTransform.Mul4(cs.Mat)
			gripCorrection := mgl64.HomogRotate3DX(math.Pi / 2).Mul4(mgl64.HomogRotate3DZ(70.0 * math.Pi / 180.0))
			corrected := absRotRobot.Mul4(gripCorrection)
			tox, toy, toz, thetaDeg = Mat4ToOVDeg(corrected)
		} else {
			calibInv := h.calibTransform.Inv()
			curRotRobot := h.calibTransform.Mul4(cs.Mat).Mul4(calibInv)
			targetRot := curRotRobot.Mul4(h.ctrlToArmOffset)
			tox, toy, toz, thetaDeg = Mat4ToOVDeg(targetRot)
		}
	} else {
		tox, toy, toz, thetaDeg = Mat4ToOVDeg(h.robotRefMat)
	}

	if math.IsNaN(tx) || math.IsNaN(tox) || math.IsNaN(thetaDeg) {
		return
	}

	rawCandidate := Pose{X: tx, Y: ty, Z: tz, OX: tox, OY: toy, OZ: toz, ThetaDeg: thetaDeg}
	candidate := EmaSmooth(h.smoothedPose, rawCandidate, h.smoothAlpha)
	h.smoothedPose = &candidate

	if !ExceedsDeadzone(h.lastSentPose, candidate, h.posDeadzone, h.rotDeadzone) {
		h.deadzoneFiltered++
		return
	}
	if h.deadzoneFiltered > 0 {
		h.svc.logger.Debugf("[%s] deadzone: suppressed %d frames", h.name, h.deadzoneFiltered)
		h.deadzoneFiltered = 0
	}

	h.lastCmdTime = now

	if h.motionSvc != nil && h.teleopActive {
		moveReq := fmt.Sprintf(
			`{"reference_frame":"world","pose":{"x":%f,"y":%f,"z":%f,"o_x":%f,"o_y":%f,"o_z":%f,"theta":%f}}`,
			candidate.X, candidate.Y, candidate.Z, candidate.OX, candidate.OY, candidate.OZ, candidate.ThetaDeg,
		)
		if h.logFile != nil {
			logEntry := fmt.Sprintf(`{"t":%d,"x":%f,"y":%f,"z":%f,"o_x":%f,"o_y":%f,"o_z":%f,"theta":%f}`+"\n",
				now.UnixMilli(), candidate.X, candidate.Y, candidate.Z, candidate.OX, candidate.OY, candidate.OZ, candidate.ThetaDeg)
			h.logFile.WriteString(logEntry)
		}
		if !h.movePending.CompareAndSwap(false, true) {
			return
		}
		h.lastSentPose = &candidate
		go func() {
			defer h.movePending.Store(false)
			if _, err := h.motionSvc.DoCommand(ctx, map[string]interface{}{
				"teleop_move": moveReq,
			}); err != nil {
				h.svc.logger.Warnf("[%s] teleop_move err: %v", h.name, err)
				h.errorTimeout = time.Now().Add(errorCooldown)
				h.sendHaptic(0.8, 200)
				if strings.Contains(err.Error(), "not running") {
					h.teleopActive = false
					h.isControlling = false
					h.svc.logger.Warnf("[%s] teleop session lost, releasing control", h.name)
				}
			}
		}()
	}
}

func (h *teleopHand) stopTeleop(ctx context.Context) {
	if h.motionSvc != nil && h.teleopActive {
		if _, err := h.motionSvc.DoCommand(ctx, map[string]interface{}{
			"teleop_stop": true,
		}); err != nil {
			h.svc.logger.Warnf("[%s] teleop_stop: %v", h.name, err)
		} else {
			h.svc.logger.Infof("[%s] teleop_stop sent", h.name)
		}
		h.teleopActive = false
	}
	if h.logFile != nil {
		h.logFile.Close()
		h.svc.logger.Infof("[%s] session log closed", h.name)
		h.logFile = nil
	}
}

func (h *teleopHand) returnToPose(ctx context.Context) {
	if len(h.poseStack) == 0 {
		return
	}
	saved := h.poseStack[len(h.poseStack)-1]
	h.poseStack = h.poseStack[:len(h.poseStack)-1]
	h.isControlling = false

	h.stopTeleop(ctx)

	componentName := h.gripperName
	if componentName == "" {
		componentName = h.armName
	}
	dest := referenceframe.NewPoseInFrame("world", spatialmath.NewPose(
		r3.Vector{X: saved.X, Y: saved.Y, Z: saved.Z},
		&spatialmath.OrientationVectorDegrees{OX: saved.OX, OY: saved.OY, OZ: saved.OZ, Theta: saved.ThetaDeg},
	))
	if h.motionSvc != nil {
		if _, err := h.motionSvc.Move(ctx, motion.MoveReq{
			ComponentName: componentName,
			Destination:   dest,
		}); err != nil {
			h.svc.logger.Warnf("[%s] returnToPose motion.Move: %v", h.name, err)
		}
	} else {
		p := spatialmath.NewPose(
			r3.Vector{X: saved.X, Y: saved.Y, Z: saved.Z},
			&spatialmath.OrientationVectorDegrees{OX: saved.OX, OY: saved.OY, OZ: saved.OZ, Theta: saved.ThetaDeg},
		)
		if err := h.arm.MoveToPosition(ctx, p, nil); err != nil {
			h.svc.logger.Warnf("[%s] returnToPose: %v", h.name, err)
		}
	}
}
