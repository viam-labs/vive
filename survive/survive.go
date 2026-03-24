// Package survive wraps libsurvive C bindings for HTC Vive tracking.
// It manages a process-global singleton with reference counting so
// multiple vive-controller instances can share a single context.
package survive

/*
#cgo CFLAGS: -I${SRCDIR}/../libsurvive/include -I${SRCDIR}/../libsurvive/include/libsurvive -I${SRCDIR}/../libsurvive/include/libsurvive/redist -DSURVIVE_ENABLE_FULL_API
#cgo LDFLAGS: -L${SRCDIR}/../libsurvive/lib -lsurvive -Wl,-rpath,${SRCDIR}/../libsurvive/lib
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
    if (msg && (strstr(msg, "OOTX") || strstr(msg, "Bad sync") || strstr(msg, "Preamble"))) return;
    printf("%s", msg);
}

// VRControllerData carries per-controller data back to Go.
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
    mat[0]  = (float)(1 - 2*(y*y + z*z));
    mat[1]  = (float)(2*(x*y - w*z));
    mat[2]  = (float)(2*(x*z + w*y));
    mat[3]  = (float)(pose->Pos[0]);
    mat[4]  = (float)(2*(x*y + w*z));
    mat[5]  = (float)(1 - 2*(x*x + z*z));
    mat[6]  = (float)(2*(y*z - w*x));
    mat[7]  = (float)(pose->Pos[1]);
    mat[8]  = (float)(2*(x*z - w*y));
    mat[9]  = (float)(2*(y*z + w*x));
    mat[10] = (float)(1 - 2*(x*x + y*y));
    mat[11] = (float)(pose->Pos[2]);
}

static int vr_init(const char *pluginPath) {
    if (pluginPath && pluginPath[0]) {
        setenv("SURVIVE_PLUGINS", pluginPath, 1);
    }
    char *args[] = {"vive"};
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

static void vr_poll_events(void) {
    if (!gCtx) return;
    SurviveSimpleEvent event;
    while (survive_simple_next_event(gCtx, &event) != SurviveSimpleEventType_None) {
    }
}

static int vr_object_count(void) {
    if (!gCtx) return 0;
    return (int)survive_simple_get_object_count(gCtx);
}

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

    int32_t buttons = survive_simple_object_get_button_mask(obj);
    if (buttons & (1 << 6))  out.pressed |= (1ULL << 1);  // menu → bit 1
    if (buttons & (1 << 7))  out.pressed |= (1ULL << 2);  // grip → bit 2
    if (buttons & (1 << 1))  out.pressed |= (1ULL << 32); // trackpad → bit 32

    out.axis1x = (float)survive_simple_object_get_input_axis(obj, 1);
    out.axis0x = (float)survive_simple_object_get_input_axis(obj, 2);
    out.axis0y = (float)survive_simple_object_get_input_axis(obj, 3);

    return out;
}

static void vr_haptic(const char *name, float amplitude, float duration_s) {
    if (!gCtx || !name) return;
    if (!survive_simple_is_running(gCtx)) return;
    SurviveSimpleObject *obj = survive_simple_get_object(gCtx, name);
    if (!obj) return;
    survive_simple_object_haptic(obj, 160.0, (FLT)amplitude, (FLT)duration_s);
}

static float vr_frame_up_z(void) {
    if (!gCtx) return 0;
    SurviveContext *ctx = survive_simple_get_ctx(gCtx);
    if (!ctx) return 0;
    // Check all lighthouses, pick the one with the strongest accel magnitude.
    // Skip any where accel hasn't been populated from OOTX yet.
    float best_z = 0;
    float best_mag = 0;
    for (int i = 0; i < NUM_GEN2_LIGHTHOUSES; i++) {
        if (!ctx->bsd[i].PositionSet) continue;
        FLT ax = ctx->bsd[i].accel[0];
        FLT ay = ctx->bsd[i].accel[1];
        FLT az = ctx->bsd[i].accel[2];
        FLT accelMag = sqrt(ax*ax + ay*ay + az*az);
        if (accelMag < 0.5) continue; // accel not yet populated
        FLT worldUp[3];
        quatrotatevector(worldUp, ctx->bsd[i].Pose.Rot, ctx->bsd[i].accel);
        if (fabs(worldUp[2]) > fabs(best_z)) {
            best_z = (float)worldUp[2];
            best_mag = (float)accelMag;
        }
    }
    return best_z;
}
*/
import "C"

import (
	"fmt"
	"sync"
	"unsafe"
)

// Button bitmask constants (internal convention, mapped in the C layer).
const (
	ButtonMenu     uint64 = 1 << 1
	ButtonGrip     uint64 = 1 << 2
	ButtonTrackpad uint64 = 1 << 32
)

// ControllerData holds the state of a single tracked controller.
type ControllerData struct {
	StateValid bool
	PoseValid  bool
	Mat        [12]float32 // row-major 3×4 tracking matrix
	Pressed    uint64      // button bitmask
	Axis0X     float64     // trackpad X
	Axis0Y     float64     // trackpad Y
	Axis1X     float64     // trigger (0–1)
}

// ObjectInfo describes a tracked object discovered by libsurvive.
type ObjectInfo struct {
	Name         string
	Serial       string
	IsController bool
}

var (
	mu       sync.Mutex
	refCount int
)

// Acquire increments the reference count and initializes libsurvive on first call.
// pluginPath should point to the libsurvive shared library or plugin directory.
func Acquire(pluginPath string) error {
	mu.Lock()
	defer mu.Unlock()
	refCount++
	if refCount == 1 {
		cPath := C.CString(pluginPath)
		defer C.free(unsafe.Pointer(cPath))
		if C.vr_init(cPath) != 0 {
			refCount--
			return fmt.Errorf("libsurvive initialization failed")
		}
	}
	return nil
}

// Release decrements the reference count and shuts down libsurvive when it reaches zero.
func Release() {
	mu.Lock()
	defer mu.Unlock()
	if refCount <= 0 {
		return
	}
	refCount--
	if refCount == 0 {
		C.vr_shutdown()
	}
}

// ForceRestart shuts down and re-initializes libsurvive regardless of refcount.
// The reference count is preserved so existing holders remain valid.
func ForceRestart(pluginPath string) error {
	mu.Lock()
	defer mu.Unlock()
	if refCount > 0 {
		C.vr_shutdown()
	}
	cPath := C.CString(pluginPath)
	defer C.free(unsafe.Pointer(cPath))
	if C.vr_init(cPath) != 0 {
		return fmt.Errorf("libsurvive re-initialization failed")
	}
	return nil
}

// IsRunning returns true if the libsurvive context is active.
func IsRunning() bool {
	return C.vr_is_running() != 0
}

// PollEvents drains the libsurvive event queue. Call once per frame.
func PollEvents() {
	C.vr_poll_events()
}

// ObjectCount returns the number of tracked objects.
func ObjectCount() int {
	return int(C.vr_object_count())
}

// GetObjectInfo returns info about the i-th tracked object.
func GetObjectInfo(idx int) ObjectInfo {
	var nameBuf [64]C.char
	var serialBuf [64]C.char
	isController := C.vr_get_object_info(C.int(idx), &nameBuf[0], 64, &serialBuf[0], 64)
	return ObjectInfo{
		Name:         C.GoString(&nameBuf[0]),
		Serial:       C.GoString(&serialBuf[0]),
		IsController: isController != 0,
	}
}

// GetController reads pose and button/axis state for a named object.
// Returns nil if the object is not found or has no valid state.
func GetController(name string) *ControllerData {
	cName := C.CString(name)
	defer C.free(unsafe.Pointer(cName))
	data := C.vr_get_controller_by_name(cName)
	if data.stateValid == 0 {
		return nil
	}
	cd := &ControllerData{
		StateValid: true,
		PoseValid:  data.poseValid != 0,
		Pressed:    uint64(data.pressed),
		Axis0X:     float64(data.axis0x),
		Axis0Y:     float64(data.axis0y),
		Axis1X:     float64(data.axis1x),
	}
	if cd.PoseValid {
		for i := 0; i < 12; i++ {
			cd.Mat[i] = float32(data.mat[i])
		}
	}
	return cd
}

// Haptic triggers a haptic pulse on the named controller.
func Haptic(name string, amplitude, durationMs float64) {
	cName := C.CString(name)
	defer C.free(unsafe.Pointer(cName))
	C.vr_haptic(cName, C.float(amplitude), C.float(durationMs/1000.0))
}

// FrameUpZ returns the Z-component of the lighthouse "up" vector
// in the world frame. Positive = Z-up (correct), negative = Z-flipped.
func FrameUpZ() float64 {
	return float64(C.vr_frame_up_z())
}
