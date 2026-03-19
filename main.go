// libsurvive → Viam VR teleop (Go, E2E).
//
// Reads Vive Wand controller poses and button states from libsurvive
// and sends arm/gripper commands directly to a Viam robot at ~90 Hz.
// No frontend, SteamVR, or WebSocket required.
//
// Build requirements:
//
//	make deps    # builds libsurvive from source into libsurvive/
//	make build   # compiles the binary
//
// Usage:
//
//	make run [HZ=90]

package main

/*
#cgo CFLAGS: -I${SRCDIR}/libsurvive/include -I${SRCDIR}/libsurvive/include/libsurvive -I${SRCDIR}/libsurvive/include/libsurvive/redist -DSURVIVE_ENABLE_FULL_API
#cgo LDFLAGS: -L${SRCDIR}/libsurvive/lib -lsurvive -Wl,-rpath,${SRCDIR}/libsurvive/lib
#include <survive_api.h>
#include <survive.h>
#include <linmath.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

static SurviveSimpleContext *gCtx = NULL;

// Log callback that suppresses noisy libsurvive output.
static void vr_log_fn(struct SurviveSimpleContext *ctx, SurviveLogLevel logLevel, const char *msg) {
    if (logLevel == SURVIVE_LOG_LEVEL_WARNING) return;
    // Filter out spammy info messages.
    if (msg && (strstr(msg, "OOTX") || strstr(msg, "Bad sync") || strstr(msg, "Preamble"))) return;
    printf("%s", msg);
}

// VRControllerData carries per-controller data back to Go.
// Layout produces the same format the Go bridge functions expect.
typedef struct {
    int      stateValid;
    int      poseValid;
    float    mat[12];        // row-major 3×4 tracking matrix
    uint64_t pressed;        // bitmask: bit1=menu, bit2=grip, bit32=trackpad
    float    axis0x, axis0y; // trackpad position
    float    axis1x;         // trigger value (0–1)
} VRControllerData;

// quat_to_mat34 converts a SurvivePose (position + wxyz quaternion)
// to a row-major 3×4 matrix.
static void quat_to_mat34(const SurvivePose *pose, float mat[12]) {
    double w = pose->Rot[0], x = pose->Rot[1], y = pose->Rot[2], z = pose->Rot[3];
    // Row 0
    mat[0]  = (float)(1 - 2*(y*y + z*z));
    mat[1]  = (float)(2*(x*y - w*z));
    mat[2]  = (float)(2*(x*z + w*y));
    mat[3]  = (float)(pose->Pos[0]);
    // Row 1
    mat[4]  = (float)(2*(x*y + w*z));
    mat[5]  = (float)(1 - 2*(x*x + z*z));
    mat[6]  = (float)(2*(y*z - w*x));
    mat[7]  = (float)(pose->Pos[1]);
    // Row 2
    mat[8]  = (float)(2*(x*z - w*y));
    mat[9]  = (float)(2*(y*z + w*x));
    mat[10] = (float)(1 - 2*(x*x + y*y));
    mat[11] = (float)(pose->Pos[2]);
}

// vr_init initialises libsurvive. pluginPath is the directory containing
// driver/poser plugins (e.g. "libsurvive/lib/libsurvive/plugins").
// Returns 0 on success.
static int vr_init(const char *pluginPath) {
    if (pluginPath && pluginPath[0]) {
        setenv("SURVIVE_PLUGINS", pluginPath, 1);
    }
    char *args[] = {"vr-teleop"};
    gCtx = survive_simple_init_with_logger(1, args, vr_log_fn);
    if (!gCtx) return 1;
    survive_simple_start_thread(gCtx);
    printf("[vr] libsurvive initialized\n");
    return 0;
}

static void vr_shutdown(void) {
    if (gCtx) {
        survive_simple_close(gCtx);
        gCtx = NULL;
    }
}

static int vr_is_running(void) {
    return gCtx && survive_simple_is_running(gCtx) ? 1 : 0;
}

// vr_poll_events drains the event queue. Call once per frame.
// Button/axis state is accumulated inside libsurvive and queried
// per-object via survive_simple_object_get_button_mask / get_input_axis.
static void vr_poll_events(void) {
    if (!gCtx) return;
    SurviveSimpleEvent event;
    while (survive_simple_next_event(gCtx, &event) != SurviveSimpleEventType_None) {
        // Draining the queue updates internal state for button/axis queries.
    }
}

// vr_object_count returns the number of tracked objects.
static int vr_object_count(void) {
    if (!gCtx) return 0;
    return (int)survive_simple_get_object_count(gCtx);
}

// vr_get_object_info writes the name, serial, and type of the i-th object.
// Returns 1 if the object is a controller (OBJECT type), 0 otherwise.
static int vr_get_object_info(int idx, char *nameBuf, int nameBufLen, char *serialBuf, int serialBufLen) {
    if (!gCtx) return 0;
    const SurviveSimpleObject *obj = survive_simple_get_first_object(gCtx);
    for (int i = 0; i < idx && obj; i++) {
        obj = survive_simple_get_next_object(gCtx, obj);
    }
    if (!obj) return 0;
    const char *name = survive_simple_object_name(obj);
    if (name) {
        strncpy(nameBuf, name, nameBufLen - 1);
        nameBuf[nameBufLen - 1] = '\0';
    }
    const char *serial = survive_simple_serial_number(obj);
    if (serial) {
        strncpy(serialBuf, serial, serialBufLen - 1);
        serialBuf[serialBufLen - 1] = '\0';
    } else {
        serialBuf[0] = '\0';
    }
    return survive_simple_object_get_type(obj) == SurviveSimpleObject_OBJECT ? 1 : 0;
}

// vr_get_controller_by_name reads pose and button/axis state for a named object.
static VRControllerData vr_get_controller_by_name(const char *name) {
    VRControllerData out;
    memset(&out, 0, sizeof(out));
    if (!gCtx) return out;

    if (!survive_simple_is_running(gCtx)) return out;
    SurviveSimpleObject *obj = survive_simple_get_object(gCtx, name);
    if (!obj) return out;

    out.stateValid = 1;

    SurvivePose pose;
    FLT timecode = survive_simple_object_get_latest_pose(obj, &pose);
    if (timecode > 0) {
        out.poseValid = 1;
        quat_to_mat34(&pose, out.mat);
    }

    // Button state via pollable queries.
    int32_t buttons = survive_simple_object_get_button_mask(obj);
    // Map libsurvive button IDs to our bitmask convention.
    // SURVIVE_BUTTON_MENU=6, SURVIVE_BUTTON_GRIP=7, SURVIVE_BUTTON_TRACKPAD=1
    if (buttons & (1 << 6))  out.pressed |= (1ULL << 1);  // menu → bit 1
    if (buttons & (1 << 7))  out.pressed |= (1ULL << 2);  // grip → bit 2
    if (buttons & (1 << 1))  out.pressed |= (1ULL << 32); // trackpad → bit 32

    // Axis state via pollable queries.
    out.axis1x = (float)survive_simple_object_get_input_axis(obj, 1); // SURVIVE_AXIS_TRIGGER
    out.axis0x = (float)survive_simple_object_get_input_axis(obj, 2); // SURVIVE_AXIS_TRACKPAD_X
    out.axis0y = (float)survive_simple_object_get_input_axis(obj, 3); // SURVIVE_AXIS_TRACKPAD_Y

    return out;
}

static void vr_haptic(const char *name, float amplitude, float duration_s) {
    if (!gCtx || !name) return;
    if (!survive_simple_is_running(gCtx)) return;

    SurviveSimpleObject *obj = survive_simple_get_object(gCtx, name);
    if (!obj) return;

    // 160 Hz is the standard Vive wand LRA motor frequency.
    int rc = survive_simple_object_haptic(obj, 160.0, (FLT)amplitude, (FLT)duration_s);
    if (rc != 0) {
        printf("[haptic] %s failed: %d\n", name, rc);
    }
}

// vr_frame_up_z returns the Z-component of the lighthouse "up" vector
// transformed into the world frame. Positive = Z-up (correct).
// Negative = Z-flipped. Returns 0 if no lighthouse data available.
static float vr_frame_up_z(void) {
    if (!gCtx) return 0;
    SurviveContext *ctx = survive_simple_get_ctx(gCtx);
    if (!ctx) return 0;
    for (int i = 0; i < NUM_GEN2_LIGHTHOUSES; i++) {
        if (!ctx->bsd[i].PositionSet) continue;
        FLT worldUp[3];
        quatrotatevector(worldUp, ctx->bsd[i].Pose.Rot, ctx->bsd[i].accel);
        return (float)worldUp[2];
    }
    return 0;
}
*/
import "C"

import (
	"context"
	"encoding/json"
	"flag"
	"fmt"
	"math"
	"os"
	"os/signal"
	"path/filepath"
	"strings"
	"sync"
	"sync/atomic"
	"syscall"
	"time"
	"unsafe"

	"github.com/go-gl/mathgl/mgl64"
	"github.com/golang/geo/r3"
	"go.viam.com/rdk/components/arm"
	"go.viam.com/rdk/components/gripper"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/robot/client"
	"go.viam.com/rdk/services/motion"
	"go.viam.com/rdk/spatialmath"
	"go.viam.com/utils/rpc"
)

// ---------------------------------------------------------------------------
// Button bitmask constants (internal convention, mapped in the C layer)
// ---------------------------------------------------------------------------

const (
	buttonMenu     uint64 = 1 << 1
	buttonGrip     uint64 = 1 << 2
	buttonTrackpad uint64 = 1 << 32
)

// ---------------------------------------------------------------------------
// 4×4 matrix math (mgl64)
// ---------------------------------------------------------------------------

// lighthouseTransform is the Lighthouse tracking universe → Viam robot frame matrix.
// Equivalent to rotZ(-90°) * rotX(90°) (same as the WebXR transform in the TS code).
// Combined with calibration yaw (applied on the right), this maps:
//
//	User forward → Robot +X, Up(+Y) → Robot +Z, User left → Robot +Y.
//
// Without calibration (user facing Lighthouse -Z):
//
//	-Z → +X, +Y → +Z, +X → +Y.
//
// lighthouseTransform maps libsurvive tracking frame → Viam robot frame.
// libsurvive uses Z-up, -Y-forward, X-left. Viam uses Z-up, X-forward, Y-left.
// A +90° rotation around Z gives: -Y→+X (forward), +X→+Y (left), +Z→+Z (up).
var lighthouseTransform = mgl64.HomogRotate3DZ(math.Pi / 2)

// mat4ToOVDeg converts a rotation matrix to a Viam orientation vector (theta in degrees).
// Extracts a quaternion from the matrix and delegates to quatToOVDeg.
func mat4ToOVDeg(m mgl64.Mat4) (ox, oy, oz, thetaDeg float64) {
	return quatToOVDeg(mgl64.Mat4ToQuat(m).Normalize())
}

// quatToOVDeg converts a unit quaternion to a Viam orientation vector (theta in degrees).
// Port of OrientationVector.setFromQuaternion from OrientationVector.ts.
func quatToOVDeg(q mgl64.Quat) (ox, oy, oz, thetaDeg float64) {
	const eps = 0.0001
	conj := q.Conjugate()
	xAxis := mgl64.Quat{W: 0, V: mgl64.Vec3{-1, 0, 0}}
	zAxis := mgl64.Quat{W: 0, V: mgl64.Vec3{0, 0, 1}}
	newX := q.Mul(xAxis).Mul(conj)
	newZ := q.Mul(zAxis).Mul(conj)

	var th float64
	nzx, nzy, nzz := newZ.V[0], newZ.V[1], newZ.V[2]
	nxx, nxy, nxz := newX.V[0], newX.V[1], newX.V[2]

	if 1-math.Abs(nzz) > eps {
		// normal1 = newZimag × newXimag
		n1x := nzy*nxz - nzz*nxy
		n1y := nzz*nxx - nzx*nxz
		n1z := nzx*nxy - nzy*nxx
		// normal2 = newZimag × (0,0,1)
		n2x := nzy
		n2y := -nzx
		n2z := 0.0
		mag1 := math.Sqrt(n1x*n1x + n1y*n1y + n1z*n1z)
		mag2 := math.Sqrt(n2x*n2x + n2y*n2y + n2z*n2z)
		if mag1 > 1e-10 && mag2 > 1e-10 {
			cosTheta := math.Max(-1, math.Min(1, (n1x*n2x+n1y*n2y+n1z*n2z)/(mag1*mag2)))
			theta := math.Acos(cosTheta)
			if theta > eps {
				magNZ := math.Sqrt(nzx*nzx + nzy*nzy + nzz*nzz)
				if magNZ > 1e-10 {
					rotQ := mgl64.QuatRotate(-theta, mgl64.Vec3{nzx / magNZ, nzy / magNZ, nzz / magNZ})
					testZ := rotQ.Mul(zAxis).Mul(rotQ.Conjugate())
					n3x := nzy*testZ.V[2] - nzz*testZ.V[1]
					n3y := nzz*testZ.V[0] - nzx*testZ.V[2]
					n3z := nzx*testZ.V[1] - nzy*testZ.V[0]
					mag3 := math.Sqrt(n3x*n3x + n3y*n3y + n3z*n3z)
					if mag3 > 1e-10 {
						cosTest := (n1x*n3x + n1y*n3y + n1z*n3z) / (mag1 * mag3)
						if 1-cosTest < eps*eps {
							th = -theta
						} else {
							th = theta
						}
					}
				}
			}
		}
	} else if nzz < 0 {
		th = -math.Atan2(nxy, nxx)
	} else {
		th = -math.Atan2(nxy, -nxx)
	}

	mag := math.Sqrt(nzx*nzx + nzy*nzy + nzz*nzz)
	if mag < 1e-10 {
		mag = 1
	}
	return nzx / mag, nzy / mag, nzz / mag, th * 180 / math.Pi
}

// ---------------------------------------------------------------------------
// Controller state
// ---------------------------------------------------------------------------

type ControllerState struct {
	Connected       bool
	Pos             [3]float64 // x, y, z (meters)
	Mat             mgl64.Mat4 // full 4×4 homogeneous transform
	Trigger         float64
	TriggerPressed  bool
	Grip            bool
	Trackpad        [2]float64
	TrackpadPressed bool
	Menu            bool
}

var nullController = ControllerState{Mat: mgl64.Ident4()}

// mat34ToMat4 converts a row-major 3×4 float32 matrix to an mgl64.Mat4.
// Input layout: m[row*4+col], mgl64 layout: column-major [col*4+row].
func mat34ToMat4(m [12]float32) mgl64.Mat4 {
	return mgl64.Mat4{
		float64(m[0]), float64(m[4]), float64(m[8]), 0, // column 0
		float64(m[1]), float64(m[5]), float64(m[9]), 0, // column 1
		float64(m[2]), float64(m[6]), float64(m[10]), 0, // column 2
		float64(m[3]), float64(m[7]), float64(m[11]), 1, // column 3 (translation + w=1)
	}
}

func readController(name string) *ControllerState {
	cName := C.CString(name)
	defer C.free(unsafe.Pointer(cName))
	data := C.vr_get_controller_by_name(cName)
	if data.stateValid == 0 {
		return nil
	}
	pressed := uint64(data.pressed)
	trigger := float64(data.axis1x)
	cs := &ControllerState{
		Connected:       data.poseValid != 0,
		Trigger:         trigger,
		TriggerPressed:  trigger > 0.8,
		Grip:            pressed&buttonGrip != 0,
		Trackpad:        [2]float64{float64(data.axis0x), -float64(data.axis0y)},
		TrackpadPressed: pressed&buttonTrackpad != 0,
		Menu:            pressed&buttonMenu != 0,
	}
	if data.poseValid != 0 {
		var flat [12]float32
		for i := 0; i < 12; i++ {
			flat[i] = float32(data.mat[i])
		}
		m := mat34ToMat4(flat)
		cs.Mat = m
		cs.Pos = [3]float64{m.At(0, 3), m.At(1, 3), m.At(2, 3)}
	}
	return cs
}

// controllerMapFile persists serial→side assignments.
const controllerMapFile = "controller_map.json"

type controllerMap struct {
	Left  string `json:"left"`  // serial number
	Right string `json:"right"` // serial number
}

func loadControllerMap(dir string) controllerMap {
	var m controllerMap
	b, err := os.ReadFile(filepath.Join(dir, controllerMapFile))
	if err != nil {
		return m
	}
	json.Unmarshal(b, &m)
	return m
}

func saveControllerMap(dir string, m controllerMap) {
	b, _ := json.MarshalIndent(m, "", "  ")
	if err := os.WriteFile(filepath.Join(dir, controllerMapFile), b, 0644); err != nil {
		fmt.Printf("[vr] failed to save controller map: %v\n", err)
	}
}

type discoveredController struct {
	name   string
	serial string
}

var (
	controllersAssigned    bool
	serialWaitLogged       bool
	firstControllerSeenAt  time.Time
)

const serialWaitTimeout = 15 * time.Second

func findControllers(leftOverride, rightOverride, calibDir string) (left, right *string) {
	if leftOverride != "" {
		left = &leftOverride
	}
	if rightOverride != "" {
		right = &rightOverride
	}
	if left != nil && right != nil {
		return
	}

	count := int(C.vr_object_count())
	var controllers []discoveredController
	for i := 0; i < count; i++ {
		var nameBuf [64]C.char
		var serialBuf [64]C.char
		isController := C.vr_get_object_info(C.int(i), &nameBuf[0], 64, &serialBuf[0], 64)
		if isController != 0 {
			controllers = append(controllers, discoveredController{
				name:   C.GoString(&nameBuf[0]),
				serial: C.GoString(&serialBuf[0]),
			})
		}
	}

	if len(controllers) == 0 {
		return
	}

	if firstControllerSeenAt.IsZero() {
		firstControllerSeenAt = time.Now()
	}

	// If overrides are set, fill in what we can and return.
	if left != nil || right != nil {
		for _, c := range controllers {
			n := c.name
			if left == nil {
				left = &n
			} else if right == nil && n != *left {
				right = &n
			}
		}
		return
	}

	// Check if all controllers have serials.
	allSerialsAvailable := true
	for _, c := range controllers {
		if c.serial == "" {
			allSerialsAvailable = false
			break
		}
	}

	// Wait for serials unless we've timed out.
	if !allSerialsAvailable && time.Since(firstControllerSeenAt) < serialWaitTimeout {
		if !serialWaitLogged {
			fmt.Println("[vr] waiting for controller serial numbers...")
			serialWaitLogged = true
		}
		return
	}

	if allSerialsAvailable && !controllersAssigned {
		for _, c := range controllers {
			fmt.Printf("[vr] controller %s serial=%s\n", c.name, c.serial)
		}
	} else if !allSerialsAvailable && !controllersAssigned {
		fmt.Println("[vr] warning: serial numbers not available, using discovery order (may be inconsistent)")
		for _, c := range controllers {
			fmt.Printf("[vr] controller %s serial=%s\n", c.name, c.serial)
		}
	}

	// Use serial-based mapping.
	cm := loadControllerMap(calibDir)

	// Build serial→name lookup.
	serialToName := map[string]string{}
	for _, c := range controllers {
		if c.serial != "" {
			serialToName[c.serial] = c.name
		}
	}

	// Try to match saved serials to discovered controllers.
	var leftName, rightName string
	if cm.Left != "" {
		if n, ok := serialToName[cm.Left]; ok {
			leftName = n
		}
	}
	if cm.Right != "" {
		if n, ok := serialToName[cm.Right]; ok {
			rightName = n
		}
	}

	// If both serials mapped to the same controller, keep left and reassign right.
	if leftName != "" && rightName != "" && leftName == rightName {
		rightName = ""
	}

	// Assign any unmatched controllers to open slots.
	assigned := map[string]bool{}
	if leftName != "" {
		assigned[leftName] = true
	}
	if rightName != "" {
		assigned[rightName] = true
	}
	for _, c := range controllers {
		if assigned[c.name] {
			continue
		}
		if leftName == "" {
			leftName = c.name
			assigned[c.name] = true
		} else if rightName == "" {
			rightName = c.name
			assigned[c.name] = true
		}
	}

	// Update and save the map with current serials.
	needSave := false
	for _, c := range controllers {
		if c.serial == "" {
			continue
		}
		if c.name == leftName && cm.Left != c.serial {
			cm.Left = c.serial
			needSave = true
		}
		if c.name == rightName && cm.Right != c.serial {
			cm.Right = c.serial
			needSave = true
		}
	}
	if needSave {
		saveControllerMap(calibDir, cm)
		fmt.Printf("[vr] controller map saved: left=%s right=%s\n", cm.Left, cm.Right)
	} else if !controllersAssigned && (cm.Left != "" || cm.Right != "") {
		fmt.Printf("[vr] controller map matched: left=%s right=%s\n", cm.Left, cm.Right)
	}

	controllersAssigned = true

	if leftName != "" {
		left = &leftName
	}
	if rightName != "" {
		right = &rightName
	}
	return
}

func hapticPulse(name string, intensity, durationMs float64) {
	cName := C.CString(name)
	defer C.free(unsafe.Pointer(cName))
	C.vr_haptic(cName, C.float(intensity), C.float(durationMs/1000.0))
}

// ---------------------------------------------------------------------------
// Calibration
// ---------------------------------------------------------------------------

var (
	calibMu  sync.RWMutex
	calibYaw float64
	calibSet bool
)

type calibData struct {
	Yaw float64 `json:"yaw"`
}

func loadCalib(dir string) {
	path := filepath.Join(dir, "calibration.json")
	b, err := os.ReadFile(path)
	if err != nil {
		fmt.Println("[teleop] No calibration file, forward = raw tracking frame")
		return
	}
	var cd calibData
	if err := json.Unmarshal(b, &cd); err != nil {
		fmt.Printf("[teleop] Bad calibration file: %v\n", err)
		return
	}
	calibMu.Lock()
	calibYaw = cd.Yaw
	calibSet = true
	calibMu.Unlock()
	fmt.Printf("[teleop] Calibration loaded: %.1f°\n", cd.Yaw*180/math.Pi)
}

func saveCalib(yaw float64, dir string) {
	calibMu.Lock()
	calibYaw = yaw
	calibSet = true
	calibMu.Unlock()
	path := filepath.Join(dir, "calibration.json")
	b, _ := json.Marshal(calibData{Yaw: yaw})
	if err := os.WriteFile(path, b, 0644); err != nil {
		fmt.Printf("[teleop] Failed to save calibration: %v\n", err)
	} else {
		fmt.Printf("[teleop] Calibrated forward: %.1f° (saved)\n", yaw*180/math.Pi)
	}
}

func computeCalibYaw(m mgl64.Mat4) (float64, bool) {
	// Controller's +Y axis (forward in libsurvive) is the second column of the rotation.
	cfx := m.At(0, 1)
	cfy := m.At(1, 1)
	mag := math.Sqrt(cfx*cfx + cfy*cfy)
	if mag < 1e-6 {
		return 0, false
	}
	// Yaw = angle from +Y to the user's forward, around +Z axis.
	return math.Atan2(-cfx/mag, cfy/mag), true
}

func applyCalib(cs ControllerState) ControllerState {
	if !cs.Connected {
		return cs
	}
	calibMu.RLock()
	yaw := calibYaw
	set := calibSet
	calibMu.RUnlock()
	if !set || yaw == 0 {
		return cs
	}
	cosY := math.Cos(yaw)
	sinY := math.Sin(yaw)
	x, y := cs.Pos[0], cs.Pos[1]
	cs.Pos = [3]float64{x*cosY - y*sinY, x*sinY + y*cosY, cs.Pos[2]}
	// Rotation is NOT modified here — calibration yaw only affects position.
	// The rotation offset is computed as a delta (curRot * inv(ctrlRef)) which
	// is independent of any global yaw, so applying yaw here would conjugate
	// the delta and change its axis incorrectly.
	return cs
}

// ---------------------------------------------------------------------------
// Teleop hand — controls one arm + optional gripper
// ---------------------------------------------------------------------------

type pose struct {
	x, y, z              float64
	ox, oy, oz, thetaDeg float64
}

// emaSmooth applies exponential moving average smoothing to a pose.
// If prev is nil (first frame), returns raw unmodified.
// alpha in (0,1]: 1 = no smoothing, lower = more smoothing.
func emaSmooth(prev *pose, raw pose, alpha float64) pose {
	if prev == nil || alpha >= 1.0 {
		return raw
	}
	b := 1.0 - alpha
	return pose{
		x:        alpha*raw.x + b*prev.x,
		y:        alpha*raw.y + b*prev.y,
		z:        alpha*raw.z + b*prev.z,
		ox:       alpha*raw.ox + b*prev.ox,
		oy:       alpha*raw.oy + b*prev.oy,
		oz:       alpha*raw.oz + b*prev.oz,
		thetaDeg: alpha*raw.thetaDeg + b*prev.thetaDeg,
	}
}

// exceedsDeadzone returns true if the new pose differs from the last-sent pose
// by more than the given position (mm) or rotation (degrees) thresholds.
// If lastSent is nil (first frame), always returns true.
func exceedsDeadzone(lastSent *pose, newPose pose, posMM, rotDeg float64) bool {
	if lastSent == nil {
		return true
	}
	if posMM <= 0 && rotDeg <= 0 {
		return true
	}

	dx := newPose.x - lastSent.x
	dy := newPose.y - lastSent.y
	dz := newPose.z - lastSent.z
	posDist := math.Sqrt(dx*dx + dy*dy + dz*dz)

	dotOV := lastSent.ox*newPose.ox + lastSent.oy*newPose.oy + lastSent.oz*newPose.oz
	dotOV = math.Max(-1, math.Min(1, dotOV))
	axisDeg := math.Acos(dotOV) * 180 / math.Pi
	thetaDiff := math.Abs(newPose.thetaDeg - lastSent.thetaDeg)
	rotDist := axisDeg + thetaDiff

	posExceeds := posMM <= 0 || posDist > posMM
	rotExceeds := rotDeg <= 0 || rotDist > rotDeg

	return posExceeds || rotExceeds
}

type teleopHand struct {
	name        string
	armName     string
	gripperName string
	arm         arm.Arm
	gripper     gripper.Gripper // nil if no gripper
	motionSvc   motion.Service
	deviceName  *string
	scale       float64
	rotEnabled  bool
	absoluteRot bool
	armFrameMat mgl64.Mat4 // arm's frame orientation from FrameSystemConfig

	// button edge state
	wasGrip bool
	wasMenu bool

	// gripper proportional control
	gripperDesired atomic.Int64 // desired position (830=open, 10=closed)
	gripperNotify  chan struct{}

	// control state
	isControlling bool
	teleopActive  bool // true between teleop_start and teleop_stop
	errorTimeout  time.Time
	lastCmdTime   time.Time
	movePending   atomic.Bool // true while a teleop_move goroutine is in flight
	poseStack     []pose

	// session logging
	logFile *os.File

	// dead-zone filtering: only send commands when pose changes enough
	posDeadzone      float64 // position dead-zone in mm
	rotDeadzone      float64 // rotation dead-zone in degrees
	lastSentPose     *pose   // nil = send next unconditionally
	deadzoneFiltered int     // count of frames suppressed since last send

	// EMA smoothing state
	smoothAlpha  float64 // EMA alpha: 1.0 = no smoothing
	smoothedPose *pose   // nil = first frame, use raw value

	// reference capture (on grip press)
	ctrlRefPos      [3]float64
	ctrlRefRotRobot mgl64.Mat4
	calibTransform  mgl64.Mat4 // lighthouseTransform * calibYaw, captured at grip press
	robotRefPos     [3]float64
	robotRefMat     mgl64.Mat4
	ctrlToArmOffset mgl64.Mat4
}

const (
	cmdInterval      = 1 * time.Millisecond // effectively no throttle — poll loop Hz is the rate limiter
	errorCooldown    = 1 * time.Second
	errorHapticIntvl = 200 * time.Millisecond
)

func newTeleopHand(name, armName, gripperName string, scale float64, rotEnabled bool, posDeadzone, rotDeadzone, smoothAlpha float64) *teleopHand {
	h := &teleopHand{
		name:            name,
		armName:         armName,
		gripperName:     gripperName,
		scale:           scale,
		rotEnabled:      rotEnabled,
		absoluteRot:     true,
		armFrameMat:     mgl64.Ident4(),
		ctrlToArmOffset: mgl64.Ident4(),
		gripperNotify:   make(chan struct{}, 1),
		posDeadzone:     posDeadzone,
		rotDeadzone:     rotDeadzone,
		smoothAlpha:     smoothAlpha,
	}
	h.gripperDesired.Store(830) // start fully open
	return h
}

func (h *teleopHand) connect(ctx context.Context, robot *client.RobotClient) error {
	a, err := arm.FromRobot(robot, h.armName)
	if err != nil {
		return fmt.Errorf("arm %q: %w", h.armName, err)
	}
	h.arm = a
	if h.gripperName != "" {
		g, err := gripper.FromRobot(robot, h.gripperName)
		if err != nil {
			fmt.Printf("[%s] gripper %q not available: %v\n", h.name, h.gripperName, err)
		} else {
			h.gripper = g
		}
	}
	// Query arm's frame orientation for absolute rotation mode.
	fsCfg, err := robot.FrameSystemConfig(ctx)
	if err != nil {
		fmt.Printf("[%s] FrameSystemConfig: %v (absolute mode may be inaccurate)\n", h.name, err)
	} else {
		for _, part := range fsCfg.Parts {
			if part.FrameConfig.Name() == h.armName {
				ori := part.FrameConfig.Pose().Orientation().Quaternion()
				q := mgl64.Quat{W: ori.Real, V: mgl64.Vec3{ori.Imag, ori.Jmag, ori.Kmag}}.Normalize()
				h.armFrameMat = q.Mat4()
				fmt.Printf("[%s] arm frame quat: (%.3f, %.3f, %.3f, %.3f)\n",
					h.name, q.V[0], q.V[1], q.V[2], q.W)
				break
			}
		}
	}
	ms, err := motion.FromProvider(robot, "builtin")
	if err != nil {
		fmt.Printf("[%s] motion service not available: %v\n", h.name, err)
	} else {
		h.motionSvc = ms
	}
	if h.gripper != nil {
		go h.gripperLoop(ctx)
	}
	if h.motionSvc != nil {
		go h.teleopStatusLoop(ctx)
	}
	return nil
}

// teleopStatusLoop polls teleop_status at 1Hz while a teleop session is active.
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
			fmt.Printf("[%s] teleop_status: %v\n", h.name, err)
			continue
		}
		// Parse {"teleop_status": {"running": bool, "error": ...}}
		if statusVal, ok := resp["teleop_status"]; ok {
			if statusMap, ok := statusVal.(map[string]interface{}); ok {
				if errVal, ok := statusMap["error"]; ok && errVal != nil {
					if errStr, ok := errVal.(string); ok && errStr != "" {
						fmt.Printf("[%s] teleop error: %s\n", h.name, errStr)
						h.sendHaptic(0.8, 200)
					}
				}
			}
		}
		fmt.Printf("[%s] teleop_status: %v\n", h.name, resp)
	}
}

// gripperLoop sends gripper DoCommand calls serially. After each round-trip
// completes it immediately sends the latest desired position, so the gripper
// is never more than one RTT behind the trigger.
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
			fmt.Printf("[%s] gripper set %d: %v\n", h.name, int(pos), err)
		}
	}
}

func (h *teleopHand) tick(ctx context.Context, cs ControllerState) {
	// --- Gripper (trigger → proportional position) ---
	if h.gripper != nil {
		// Map trigger 0.0–1.0 to gripper position 830 (open) – 10 (closed).
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

	// Arm requires connected pose.
	if !cs.Connected {
		return
	}

	// --- Grip: start/stop arm control ---
	if cs.Grip && !h.wasGrip {
		h.startControl(ctx, cs)
	} else if !cs.Grip && h.wasGrip && h.isControlling {
		h.isControlling = false
		h.sendHaptic(0.3, 80)
		go h.stopTeleop(ctx)
	}
	h.wasGrip = cs.Grip

	// --- Menu: return to saved pose ---
	if cs.Menu && !h.wasMenu && len(h.poseStack) > 0 {
		go h.returnToPose(ctx)
	}
	h.wasMenu = cs.Menu

	// --- Control frame ---
	if h.isControlling {
		h.controlFrame(ctx, cs)
	}
}

// teleopComponentName returns the component name string for motion service
// teleop commands (gripper if available, otherwise arm).
func (h *teleopHand) teleopComponentName() string {
	if h.gripperName != "" {
		return h.gripperName
	}
	return h.armName
}

func (h *teleopHand) startControl(ctx context.Context, cs ControllerState) {
	go func() {
		// Get current pose of the gripper (or arm) in world frame via motion service.
		componentName := h.gripperName
		if componentName == "" {
			componentName = h.armName
		}
		var currentPose spatialmath.Pose
		if h.motionSvc != nil {
			pif, err := h.motionSvc.GetPose(ctx, componentName, "world", nil, nil)
			if err != nil {
				fmt.Printf("[%s] startControl: GetPose: %v (falling back to arm.EndPosition)\n", h.name, err)
				ep, err2 := h.arm.EndPosition(ctx, nil)
				if err2 != nil {
					fmt.Printf("[%s] startControl: EndPosition: %v\n", h.name, err2)
					return
				}
				currentPose = ep
			} else {
				currentPose = pif.Pose()
			}
		} else {
			ep, err := h.arm.EndPosition(ctx, nil)
			if err != nil {
				fmt.Printf("[%s] startControl: EndPosition: %v\n", h.name, err)
				return
			}
			currentPose = ep
		}

		pt := currentPose.Point()
		ori := currentPose.Orientation()
		ovd := ori.OrientationVectorDegrees()

		h.robotRefPos = [3]float64{pt.X, pt.Y, pt.Z}

		// Reconstruct robot arm rotation matrix from orientation vector degrees.
		ovRad := spatialmath.NewOrientationVector()
		ovRad.OX = ovd.OX
		ovRad.OY = ovd.OY
		ovRad.OZ = ovd.OZ
		ovRad.Theta = ovd.Theta * math.Pi / 180
		rq := ovRad.Quaternion()
		h.robotRefMat = mgl64.Quat{W: rq.Real, V: mgl64.Vec3{rq.Imag, rq.Jmag, rq.Kmag}}.Normalize().Mat4()

		h.poseStack = append(h.poseStack, pose{
			x: pt.X, y: pt.Y, z: pt.Z,
			ox: ovd.OX, oy: ovd.OY, oz: ovd.OZ, thetaDeg: ovd.Theta,
		})

		h.ctrlRefPos = cs.Pos

		// Build calibrated rotation transform: lighthouseTransform * yawM.
		calibMu.RLock()
		yaw := calibYaw
		calibMu.RUnlock()
		h.calibTransform = lighthouseTransform
		if yaw != 0 {
			yawM := mgl64.HomogRotate3DZ(yaw)
			h.calibTransform = lighthouseTransform.Mul4(yawM)
		}

		// Similarity transform: T * M * T^-1 maps controller rotation to robot frame.
		calibInv := h.calibTransform.Inv()
		h.ctrlRefRotRobot = h.calibTransform.Mul4(cs.Mat).Mul4(calibInv)

		// Offset = inverse(ctrlRefRotRobot) * robotRefMat
		h.ctrlToArmOffset = h.ctrlRefRotRobot.Inv().Mul4(h.robotRefMat)

		// Send teleop_start to the motion service.
		if h.motionSvc != nil {
			compName := h.teleopComponentName()
			startReq := fmt.Sprintf(
				`{"name":"builtin","component_name":"%s","destination":{"reference_frame":"world","pose":{"x":%f,"y":%f,"z":%f,"o_x":%f,"o_y":%f,"o_z":%f,"theta":%f}}}`,
				compName, pt.X, pt.Y, pt.Z, ovd.OX, ovd.OY, ovd.OZ, ovd.Theta,
			)
			fmt.Printf("[%s] teleop_start payload: %s\n", h.name, startReq)
			if _, err := h.motionSvc.DoCommand(ctx, map[string]interface{}{
				"teleop_start": startReq,
			}); err != nil {
				fmt.Printf("[%s] teleop_start failed: %v\n", h.name, err)
				h.sendHaptic(0.8, 200)
				return
			}
			h.teleopActive = true
			fmt.Printf("[%s] teleop_start sent for %s\n", h.name, compName)

			// Open session log file.
			logName := fmt.Sprintf("teleop_%s_%s.jsonl", h.name, time.Now().Format("20060102_150405"))
			if f, err := os.Create(logName); err != nil {
				fmt.Printf("[%s] failed to create log file %s: %v\n", h.name, logName, err)
			} else {
				h.logFile = f
				fmt.Printf("[%s] logging poses to %s\n", h.name, logName)
			}
		}

		h.errorTimeout = time.Time{}
		h.lastSentPose = nil
		h.smoothedPose = nil
		h.isControlling = true
		h.sendHaptic(0.5, 100)
		fmt.Printf("[%s] control started at (%.1f, %.1f, %.1f)\n", h.name, pt.X, pt.Y, pt.Z)
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

	// Position: delta in Lighthouse space → rotate by calibTransform → robot frame.
	dx := cs.Pos[0] - h.ctrlRefPos[0]
	dy := cs.Pos[1] - h.ctrlRefPos[1]
	dz := cs.Pos[2] - h.ctrlRefPos[2]
	delta := h.calibTransform.Mul4x1(mgl64.Vec4{dx, dy, dz, 0})
	scaleMM := h.scale * 1000
	tx := h.robotRefPos[0] + delta[0]*scaleMM
	ty := h.robotRefPos[1] + delta[1]*scaleMM
	tz := h.robotRefPos[2] + delta[2]*scaleMM

	// Rotation.
	var tox, toy, toz, thetaDeg float64
	if h.rotEnabled {
		if h.absoluteRot {
			absRotRobot := h.calibTransform.Mul4(cs.Mat)
			gripCorrection := mgl64.HomogRotate3DX(math.Pi / 2).Mul4(mgl64.HomogRotate3DZ(70.0 * math.Pi / 180.0))
			corrected := absRotRobot.Mul4(gripCorrection)
			tox, toy, toz, thetaDeg = mat4ToOVDeg(corrected)
		} else {
			calibInv := h.calibTransform.Inv()
			curRotRobot := h.calibTransform.Mul4(cs.Mat).Mul4(calibInv)
			targetRot := curRotRobot.Mul4(h.ctrlToArmOffset)
			tox, toy, toz, thetaDeg = mat4ToOVDeg(targetRot)
		}
	} else {
		tox, toy, toz, thetaDeg = mat4ToOVDeg(h.robotRefMat)
	}

	if math.IsNaN(tx) || math.IsNaN(tox) || math.IsNaN(thetaDeg) {
		return
	}

	// EMA smoothing (before deadzone filter).
	rawCandidate := pose{x: tx, y: ty, z: tz, ox: tox, oy: toy, oz: toz, thetaDeg: thetaDeg}
	candidate := emaSmooth(h.smoothedPose, rawCandidate, h.smoothAlpha)
	h.smoothedPose = &candidate

	// Dead-zone filter: suppress command if movement is below threshold.
	if !exceedsDeadzone(h.lastSentPose, candidate, h.posDeadzone, h.rotDeadzone) {
		h.deadzoneFiltered++
		return
	}
	if h.deadzoneFiltered > 0 {
		fmt.Printf("[%s] deadzone: suppressed %d frames\n", h.name, h.deadzoneFiltered)
		h.deadzoneFiltered = 0
	}

	h.lastCmdTime = now

	// teleop_move is non-blocking server-side — fire async so poll loop
	// doesn't block on gRPC round-trip.
	if h.motionSvc != nil && h.teleopActive {
		moveReq := fmt.Sprintf(
			`{"reference_frame":"world","pose":{"x":%f,"y":%f,"z":%f,"o_x":%f,"o_y":%f,"o_z":%f,"theta":%f}}`,
			candidate.x, candidate.y, candidate.z, candidate.ox, candidate.oy, candidate.oz, candidate.thetaDeg,
		)
		// fmt.Printf("[%s] teleop_move: %s\n", h.name, moveReq)
		if h.logFile != nil {
			logEntry := fmt.Sprintf(`{"t":%d,"x":%f,"y":%f,"z":%f,"o_x":%f,"o_y":%f,"o_z":%f,"theta":%f}`+"\n",
				now.UnixMilli(), candidate.x, candidate.y, candidate.z, candidate.ox, candidate.oy, candidate.oz, candidate.thetaDeg)
			h.logFile.WriteString(logEntry)
		}
		if !h.movePending.CompareAndSwap(false, true) {
			return // prior teleop_move still in flight; skip this frame
		}
		h.lastSentPose = &candidate
		go func() {
			defer h.movePending.Store(false)
			if _, err := h.motionSvc.DoCommand(ctx, map[string]interface{}{
				"teleop_move": moveReq,
			}); err != nil {
				fmt.Printf("[%s] teleop_move err: %v\n", h.name, err)
				h.errorTimeout = time.Now().Add(errorCooldown)
				h.sendHaptic(0.8, 200)
				// If the server-side pipeline died, stop sending moves.
				if strings.Contains(err.Error(), "not running") {
					h.teleopActive = false
					h.isControlling = false
					fmt.Printf("[%s] teleop session lost, releasing control\n", h.name)
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
			fmt.Printf("[%s] teleop_stop: %v\n", h.name, err)
		} else {
			fmt.Printf("[%s] teleop_stop sent\n", h.name)
		}
		h.teleopActive = false
	}
	if h.logFile != nil {
		h.logFile.Close()
		fmt.Printf("[%s] session log closed\n", h.name)
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

	// Stop any active teleop session before returning to pose.
	h.stopTeleop(ctx)

	componentName := h.gripperName
	if componentName == "" {
		componentName = h.armName
	}
	dest := referenceframe.NewPoseInFrame("world", spatialmath.NewPose(
		r3.Vector{X: saved.x, Y: saved.y, Z: saved.z},
		&spatialmath.OrientationVectorDegrees{OX: saved.ox, OY: saved.oy, OZ: saved.oz, Theta: saved.thetaDeg},
	))
	if h.motionSvc != nil {
		if _, err := h.motionSvc.Move(ctx, motion.MoveReq{
			ComponentName: componentName,
			Destination:   dest,
		}); err != nil {
			fmt.Printf("[%s] returnToPose motion.Move: %v\n", h.name, err)
		}
	} else {
		p := spatialmath.NewPose(
			r3.Vector{X: saved.x, Y: saved.y, Z: saved.z},
			&spatialmath.OrientationVectorDegrees{OX: saved.ox, OY: saved.oy, OZ: saved.oz, Theta: saved.thetaDeg},
		)
		if err := h.arm.MoveToPosition(ctx, p, nil); err != nil {
			fmt.Printf("[%s] returnToPose: %v\n", h.name, err)
		}
	}
}

func (h *teleopHand) sendHaptic(intensity, durationMs float64) {
	if h.deviceName != nil {
		hapticPulse(*h.deviceName, intensity, durationMs)
	}
}

func (h *teleopHand) toggleRotMode() {
	h.absoluteRot = !h.absoluteRot
	mode := "relative"
	if h.absoluteRot {
		mode = "absolute"
	}
	fmt.Printf("[%s] rotation mode: %s\n", h.name, mode)
	h.sendHaptic(0.3, 80)
}

// ---------------------------------------------------------------------------
// Poll loop
// ---------------------------------------------------------------------------

func initVR(pluginPath string) int {
	cPath := C.CString(pluginPath)
	defer C.free(unsafe.Pointer(cPath))
	return int(C.vr_init(cPath))
}

func pollLoop(ctx context.Context, hz int, calibDir, pluginLib, leftCtrl, rightCtrl string, left, right *teleopHand) {
	interval := time.Duration(float64(time.Second) / float64(hz))
	fmt.Printf("[teleop] Polling at %d Hz (%.1f ms)\n", hz, float64(interval)/float64(time.Millisecond))

	loadCalib(calibDir)

	var leftName, rightName *string
	var wasLeftTrackpad, wasRightTrackpad bool
	var frameChecked bool
	lastScan := time.Time{}
	vrOK := initVR(pluginLib) == 0
	if vrOK {
		fmt.Println("[teleop] libsurvive initialized")
	} else {
		fmt.Fprintln(os.Stderr, "[teleop] libsurvive not available, will retry...")
	}

	for {
		start := time.Now()

		if time.Since(lastScan) > 2*time.Second {
			if !vrOK && initVR(pluginLib) == 0 {
				vrOK = true
				fmt.Println("[teleop] libsurvive initialized")
			}
			// Check frame orientation once after libsurvive solves base station positions.
			if vrOK && !frameChecked {
				upZ := float64(C.vr_frame_up_z())
				if upZ != 0 {
					frameChecked = true
					if upZ < 0 {
						fmt.Printf("[teleop] WARNING: frame Z-flip detected (upZ=%.2f), applying 180° X correction\n", upZ)
						lighthouseTransform = mgl64.HomogRotate3DX(math.Pi).Mul4(lighthouseTransform)
					} else {
						fmt.Printf("[teleop] frame orientation OK (upZ=%.2f)\n", upZ)
					}
				}
			}
			leftName, rightName = findControllers(leftCtrl, rightCtrl, calibDir)
			if left != nil {
				left.deviceName = leftName
			}
			if right != nil {
				right.deviceName = rightName
			}
			lastScan = time.Now()
		}

		C.vr_poll_events()

		resolve := func(name *string) ControllerState {
			if name == nil {
				return nullController
			}
			cs := readController(*name)
			if cs == nil {
				return nullController
			}
			return *cs
		}

		leftRaw := resolve(leftName)
		rightRaw := resolve(rightName)

		// Trackpad rising edge: dispatch by region.
		for _, tr := range []struct {
			raw ControllerState
			was *bool
		}{
			{leftRaw, &wasLeftTrackpad},
			{rightRaw, &wasRightTrackpad},
		} {
			pressed := tr.raw.Connected && tr.raw.TrackpadPressed
			if pressed && !*tr.was {
				y := tr.raw.Trackpad[1]
				if y < -0.3 {
					// Up: calibrate forward direction.
					if yaw, ok := computeCalibYaw(tr.raw.Mat); ok {
						saveCalib(yaw, calibDir)
						left.sendHaptic(0.3, 80)
						right.sendHaptic(0.3, 80)
					}
				} else if y > 0.3 {
					// Down: toggle rotation control mode.
					if left != nil {
						left.toggleRotMode()
					}
					if right != nil {
						right.toggleRotMode()
					}
				}
			}
			*tr.was = pressed
		}

		// Pass raw (uncalibrated) controller state to teleop hands.
		// The teleop hand uses calibTransform for both position and rotation,
		// matching the TS/WebXR approach where qTransform handles everything.
		if left != nil && left.arm != nil {
			left.tick(ctx, leftRaw)
		}
		if right != nil && right.arm != nil {
			right.tick(ctx, rightRaw)
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
// Main
// ---------------------------------------------------------------------------

func main() {
	hz := flag.Int("hz", 90, "Polling rate in Hz")
	address := flag.String("address", "", "Viam machine address")
	keyID := flag.String("key-id", "", "Viam API key ID")
	key := flag.String("key", "", "Viam API key")
	// Left controller → right-arm/gripper; right controller → left-arm/gripper.
	leftArm := flag.String("left-arm", "right-arm", "Arm controlled by the left controller")
	rightArm := flag.String("right-arm", "left-arm", "Arm controlled by the right controller")
	leftGripper := flag.String("left-gripper", "right-gripper", "Gripper controlled by the left controller (empty to disable)")
	rightGripper := flag.String("right-gripper", "left-gripper", "Gripper controlled by the right controller (empty to disable)")
	scale := flag.Float64("scale", 1.0, "Position scale factor (0.1–3.0)")
	rotEnabled := flag.Bool("rotation", true, "Enable orientation tracking")
	leftCtrl := flag.String("left-controller", "", "libsurvive object name for left controller (e.g. WM0)")
	rightCtrl := flag.String("right-controller", "", "libsurvive object name for right controller (e.g. WM1)")
	posDeadzone := flag.Float64("pos-deadzone", 0.5, "Position dead-zone in mm (0 to disable)")
	rotDeadzone := flag.Float64("rot-deadzone", 1.0, "Rotation dead-zone in degrees (0 to disable)")
	smoothAlpha := flag.Float64("smooth-alpha", 0.5, "EMA smoothing alpha (0-1, 1=no smoothing)")
	flag.Parse()

	exePath, err := os.Executable()
	if err != nil {
		fmt.Fprintf(os.Stderr, "[teleop] Cannot determine executable path: %v\n", err)
		os.Exit(1)
	}
	calibDir := filepath.Dir(exePath)

	ctx, cancel := context.WithCancel(context.Background())
	defer cancel()

	// Connect to Viam robot.
	logger := logging.NewLogger("teleop")
	fmt.Printf("[teleop] Connecting to %s ...\n", *address)

	dialOpts := []rpc.DialOption{}
	if *keyID != "" && *key != "" {
		dialOpts = append(dialOpts, rpc.WithEntityCredentials(*keyID, rpc.Credentials{
			Type:    rpc.CredentialsTypeAPIKey,
			Payload: *key,
		}))
	}

	robot, err := client.New(ctx, *address, logger, client.WithDialOptions(dialOpts...))
	if err != nil {
		fmt.Fprintf(os.Stderr, "[teleop] Failed to connect to robot: %v\n", err)
		os.Exit(1)
	}
	defer robot.Close(ctx)
	fmt.Println("[teleop] Connected to robot")

	left := newTeleopHand("left", *leftArm, *leftGripper, *scale, *rotEnabled, *posDeadzone, *rotDeadzone, *smoothAlpha)
	if err := left.connect(ctx, robot); err != nil {
		fmt.Fprintf(os.Stderr, "[teleop] Left hand: %v\n", err)
	}

	right := newTeleopHand("right", *rightArm, *rightGripper, *scale, *rotEnabled, *posDeadzone, *rotDeadzone, *smoothAlpha)
	if err := right.connect(ctx, robot); err != nil {
		fmt.Fprintf(os.Stderr, "[teleop] Right hand: %v\n", err)
	}

	defer func() {
		C.vr_shutdown()
		fmt.Println("[teleop] libsurvive shut down")
	}()

	// SURVIVE_PLUGINS must point to the .so/.dylib file; libsurvive searches
	// for plugins/ and libsurvive/plugins/ relative to its directory.
	pluginLib := filepath.Join(calibDir, "libsurvive", "lib", "libsurvive.so")
	if _, err := os.Stat(pluginLib); err != nil {
		pluginLib = filepath.Join(calibDir, "libsurvive", "lib", "libsurvive.dylib")
	}
	go pollLoop(ctx, *hz, calibDir, pluginLib, *leftCtrl, *rightCtrl, left, right)

	sig := make(chan os.Signal, 1)
	signal.Notify(sig, syscall.SIGINT, syscall.SIGTERM)
	<-sig
	fmt.Println("\n[teleop] Stopped")
	cancel()

	// Force exit if libsurvive cleanup hangs.
	go func() {
		time.Sleep(2 * time.Second)
		fmt.Fprintln(os.Stderr, "[teleop] Forced exit (libsurvive cleanup hung)")
		os.Exit(1)
	}()
}
