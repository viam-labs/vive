package vive

import (
	"math"

	"github.com/go-gl/mathgl/mgl64"
	"go.viam.com/rdk/resource"
)

var (
	family         = resource.ModelNamespace("viam").WithFamily("vive")
	ViveController = family.WithModel("vive-controller")
	ViveTeleop     = family.WithModel("teleop")
	CaptureControl = family.WithModel("capture-control")
)

// ControllerState is the processed state of a single VR controller,
// with the raw tracking data converted to usable Go types.
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

var NullController = ControllerState{Mat: mgl64.Ident4()}

// BaseLighthouseTransform maps libsurvive tracking frame → Viam robot frame.
// libsurvive uses Z-up, -Y-forward, X-left. Viam uses Z-up, X-forward, Y-left.
// A +90° rotation around Z gives: -Y→+X (forward), +X→+Y (left), +Z→+Z (up).
// This is the base transform without Z-flip correction; per-service lhTransform
// includes the Z-flip and is the value used at runtime.
var BaseLighthouseTransform = mgl64.HomogRotate3DZ(math.Pi / 2)

// Mat34ToMat4 converts a row-major 3×4 float32 matrix to an mgl64.Mat4.
// Input layout: m[row*4+col], mgl64 layout: column-major [col*4+row].
func Mat34ToMat4(m [12]float32) mgl64.Mat4 {
	return mgl64.Mat4{
		float64(m[0]), float64(m[4]), float64(m[8]), 0,
		float64(m[1]), float64(m[5]), float64(m[9]), 0,
		float64(m[2]), float64(m[6]), float64(m[10]), 0,
		float64(m[3]), float64(m[7]), float64(m[11]), 1,
	}
}

// Mat4ToOVDeg converts a rotation matrix to a Viam orientation vector (theta in degrees).
func Mat4ToOVDeg(m mgl64.Mat4) (ox, oy, oz, thetaDeg float64) {
	return QuatToOVDeg(mgl64.Mat4ToQuat(m).Normalize())
}

// QuatToOVDeg converts a unit quaternion to a Viam orientation vector (theta in degrees).
func QuatToOVDeg(q mgl64.Quat) (ox, oy, oz, thetaDeg float64) {
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
		n1x := nzy*nxz - nzz*nxy
		n1y := nzz*nxx - nzx*nxz
		n1z := nzx*nxy - nzy*nxx
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

// Pose holds a position + orientation vector for teleop commands.
type Pose struct {
	X, Y, Z              float64
	OX, OY, OZ, ThetaDeg float64
}

// EmaSmooth applies exponential moving average smoothing to a pose.
func EmaSmooth(prev *Pose, raw Pose, alpha float64) Pose {
	if prev == nil || alpha >= 1.0 {
		return raw
	}
	b := 1.0 - alpha
	return Pose{
		X:        alpha*raw.X + b*prev.X,
		Y:        alpha*raw.Y + b*prev.Y,
		Z:        alpha*raw.Z + b*prev.Z,
		OX:       alpha*raw.OX + b*prev.OX,
		OY:       alpha*raw.OY + b*prev.OY,
		OZ:       alpha*raw.OZ + b*prev.OZ,
		ThetaDeg: alpha*raw.ThetaDeg + b*prev.ThetaDeg,
	}
}

// ValidateFrameUp checks whether the controller's "up" direction (body Y-axis,
// through the tracking ring) maps to +Z in robot frame after the lighthouse transform.
// Returns true if the frame orientation is correct, false if inverted.
func ValidateFrameUp(controllerMat, lhTransform mgl64.Mat4) bool {
	// Column 1 of the controller's pose matrix is its body Y-axis in the LS world frame.
	upInLS := mgl64.Vec4{
		controllerMat.At(0, 1),
		controllerMat.At(1, 1),
		controllerMat.At(2, 1),
		0,
	}
	robotUp := lhTransform.Mul4x1(upInLS)
	return robotUp[2] > 0
}

// ExceedsDeadzone returns true if the new pose differs from the last-sent pose
// by more than the given position (mm) or rotation (degrees) thresholds.
func ExceedsDeadzone(lastSent *Pose, newPose Pose, posMM, rotDeg float64) bool {
	if lastSent == nil {
		return true
	}
	if posMM <= 0 && rotDeg <= 0 {
		return true
	}

	dx := newPose.X - lastSent.X
	dy := newPose.Y - lastSent.Y
	dz := newPose.Z - lastSent.Z
	posDist := math.Sqrt(dx*dx + dy*dy + dz*dz)

	dotOV := lastSent.OX*newPose.OX + lastSent.OY*newPose.OY + lastSent.OZ*newPose.OZ
	dotOV = math.Max(-1, math.Min(1, dotOV))
	axisDeg := math.Acos(dotOV) * 180 / math.Pi
	thetaDiff := math.Abs(newPose.ThetaDeg - lastSent.ThetaDeg)
	rotDist := axisDeg + thetaDiff

	posExceeds := posMM <= 0 || posDist > posMM
	rotExceeds := rotDeg <= 0 || rotDist > rotDeg

	return posExceeds || rotExceeds
}
