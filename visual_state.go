package vive

import (
	"time"

	"github.com/go-gl/mathgl/mgl64"
)

// HandVisual is an immutable snapshot of one hand's state for visualization.
// The control loop publishes these; the visualizer only ever loads them.
type HandVisual struct {
	Name        string
	PublishedAt time.Time // for staleness detection; see VisualizerConfig.StaleAfterMS
	Connected   bool
	Controlling bool

	ControllerPos [3]float64 // lighthouse frame, METRES (converted at render time)
	ControllerMat mgl64.Mat4

	// Commanded is the last pose sent to the motion service, in the robot world
	// frame. nil unless this hand is actively controlling.
	Commanded *Pose
}

// VisualState is the whole-module snapshot the visualizer renders.
type VisualState struct {
	Hands []HandVisual

	// ComposedCalib is ALREADY lhTransform · Rz(CalibYaw). Never re-apply
	// CalibYaw to it — that yields a frame rotated by twice the yaw, which is
	// wrong in precisely the feature meant to catch a wrong yaw.
	ComposedCalib mgl64.Mat4

	// CalibYaw is carried for display and logging only.
	CalibYaw float64
}

// publishVisual stores a fresh snapshot. Called only from the poll goroutine, so
// the unsynchronised reads of isControlling/lastSentPose are no worse than the
// existing reads on that goroutine.
func (h *teleopHand) publishVisual(cs ControllerState) {
	v := &HandVisual{
		Name:          h.name,
		PublishedAt:   time.Now(),
		Connected:     cs.Connected,
		Controlling:   h.isControlling,
		ControllerPos: cs.Pos,
		ControllerMat: cs.Mat,
	}
	if h.isControlling && h.lastSentPose != nil {
		p := *h.lastSentPose // copy: the control loop keeps mutating the original
		v.Commanded = &p
	}
	h.visual.Store(v)
}

// publishDisconnected records that this hand has no tracking data at all, so the
// visualizer removes its geometry rather than freezing it at the last pose.
func (h *teleopHand) publishDisconnected() {
	h.visual.Store(&HandVisual{Name: h.name, PublishedAt: time.Now()})
}

// VisualState snapshots every hand plus the live calibration.
//
// ok is false once the service has been closed, which is how a visualizer
// holding a handle to a rebuilt teleop service detects that it is stale —
// teleopService embeds resource.AlwaysRebuild, so reconfigure always produces a
// new instance and closes this one.
func (svc *teleopService) VisualState() (VisualState, bool) {
	if svc.closed.Load() {
		return VisualState{}, false
	}

	svc.calibMu.RLock()
	lht, yaw := svc.lhTransform, svc.calibYaw
	svc.calibMu.RUnlock()

	// Same composition startControl performs, so the rendered frame matches the
	// transform teleop actually applies.
	composed := lht
	if yaw != 0 {
		composed = lht.Mul4(mgl64.HomogRotate3DZ(yaw))
	}

	out := VisualState{ComposedCalib: composed, CalibYaw: yaw}
	for _, h := range svc.hands {
		// A nil pointer means this hand has never ticked. Omit it — a zero value
		// would render at the origin, indistinguishable from a real reading.
		if v := h.visual.Load(); v != nil {
			out.Hands = append(out.Hands, *v)
		}
	}
	return out, true
}
