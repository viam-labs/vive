package vive

import (
	"fmt"
	"time"

	"github.com/go-gl/mathgl/mgl64"
	"github.com/golang/geo/r3"
	commonpb "go.viam.com/api/common/v1"
	pb "go.viam.com/api/service/worldstatestore/v1"
	"go.viam.com/rdk/services/worldstatestore"
	"go.viam.com/rdk/spatialmath"
	"gonum.org/v1/gonum/num/quat"
)

// VisualizerConfig configures the world-state visualizer service.
type VisualizerConfig struct {
	TeleopService      string     `json:"teleop_service"`
	PublishRateHz      float64    `json:"publish_rate_hz,omitempty"`
	LighthouseOffsetMM [3]float64 `json:"lighthouse_offset_mm,omitempty"`
	StaleAfterMS       int        `json:"stale_after_ms,omitempty"`
}

const (
	frameLighthouse = "vive-lighthouse"

	controllerRadiusMM = 40.0

	// Lighthouse box dimensions. The spike proved metadata.color is ignored, so
	// orientation has to be readable from proportion alone: three mutually
	// distinguishable extents, longest on X, shortest on Z.
	//
	// One box, not a three-bar gizmo: commonpb.Transform carries a single
	// PhysicalObject, so three bars would mean three transforms and would triple
	// the object count for one frame.
	lighthouseXMM = 300.0
	lighthouseYMM = 200.0
	lighthouseZMM = 120.0

	commandedBoxMM = 60.0
)

func controllerFrameName(hand string) string { return "vive-" + hand + "-controller" }
func commandedFrameName(hand string) string  { return "vive-" + hand + "-commanded" }

// buildTransforms renders a snapshot as the set of transforms the app should show.
//
// Pure: now is a parameter rather than a clock read, so staleness is testable.
// Absence is meaningful — a hand omitted here becomes a REMOVED downstream, which
// is how "tracking died" is distinguished from "hand held still" in a scene with
// no colour available.
func buildTransforms(st VisualState, cfg VisualizerConfig, now time.Time) map[string]*commonpb.Transform {
	out := make(map[string]*commonpb.Transform, 1+2*len(st.Hands))

	// ComposedCalib is already lhTransform · Rz(yaw); do NOT apply CalibYaw again.
	out[frameLighthouse] = newTransform(
		frameLighthouse, "world",
		poseFrom(r3.Vector{
			X: cfg.LighthouseOffsetMM[0],
			Y: cfg.LighthouseOffsetMM[1],
			Z: cfg.LighthouseOffsetMM[2],
		}, st.ComposedCalib),
		boxGeometry(lighthouseXMM, lighthouseYMM, lighthouseZMM, frameLighthouse),
	)

	staleAfter := time.Duration(cfg.StaleAfterMS) * time.Millisecond
	for _, h := range st.Hands {
		if !h.Connected || now.Sub(h.PublishedAt) > staleAfter {
			continue
		}

		ctrlName := controllerFrameName(h.Name)
		out[ctrlName] = newTransform(
			ctrlName, frameLighthouse,
			// Metres -> millimetres. Never multiplied by teleop's scale factor.
			poseFrom(r3.Vector{
				X: h.ControllerPos[0] * 1000,
				Y: h.ControllerPos[1] * 1000,
				Z: h.ControllerPos[2] * 1000,
			}, h.ControllerMat),
			sphereGeometry(controllerRadiusMM, ctrlName),
		)

		if h.Controlling && h.Commanded != nil {
			cmdName := commandedFrameName(h.Name)
			out[cmdName] = newTransform(
				cmdName, "world",
				// Already in the robot world frame, in mm and degrees.
				&commonpb.Pose{
					X: h.Commanded.X, Y: h.Commanded.Y, Z: h.Commanded.Z,
					OX: h.Commanded.OX, OY: h.Commanded.OY, OZ: h.Commanded.OZ,
					Theta: h.Commanded.ThetaDeg,
				},
				boxGeometry(commandedBoxMM, commandedBoxMM, commandedBoxMM, cmdName),
			)
		}
	}
	return out
}

// newTransform assembles one transform. The UUID is the frame name's bytes:
// deterministic, stable for the resource's lifetime, and legible in logs. The
// name is set on both ReferenceFrame and the geometry Label because the app
// surfaces names but which field drives that is unconfirmed.
func newTransform(name, parent string, pose *commonpb.Pose, geom *commonpb.Geometry) *commonpb.Transform {
	return &commonpb.Transform{
		ReferenceFrame: name,
		PoseInObserverFrame: &commonpb.PoseInFrame{
			ReferenceFrame: parent,
			Pose:           pose,
		},
		PhysicalObject: geom,
		Uuid:           []byte(name),
	}
}

// poseFrom converts a position and an mgl64 rotation matrix into a Viam pose.
// Uses spatialmath rather than hand-rolled orientation-vector maths — a
// hand-rolled copy is exactly how an axis gets silently flipped, and this tool
// exists to catch flipped axes.
func poseFrom(pos r3.Vector, m mgl64.Mat4) *commonpb.Pose {
	q := mgl64.Mat4ToQuat(m).Normalize()
	ovd := spatialmath.QuatToOVD(quat.Number{
		Real: q.W, Imag: q.V[0], Jmag: q.V[1], Kmag: q.V[2],
	})
	return spatialmath.PoseToProtobuf(spatialmath.NewPose(pos, ovd))
}

func boxGeometry(x, y, z float64, label string) *commonpb.Geometry {
	return &commonpb.Geometry{
		Center: &commonpb.Pose{},
		GeometryType: &commonpb.Geometry_Box{
			Box: &commonpb.RectangularPrism{
				DimsMm: &commonpb.Vector3{X: x, Y: y, Z: z},
			},
		},
		Label: label,
	}
}

func sphereGeometry(radiusMM float64, label string) *commonpb.Geometry {
	return &commonpb.Geometry{
		Center: &commonpb.Pose{},
		GeometryType: &commonpb.Geometry_Sphere{
			Sphere: &commonpb.Sphere{RadiusMm: radiusMM},
		},
		Label: label,
	}
}

// diffTransforms produces the change list that takes a consumer from prev to next.
//
// ADDED and REMOVED carry the full transform and no field mask. UPDATED carries a
// partial transform — UUID plus only the changed fields — with lowerCamelCase
// dotted field paths, matching the RDK fake, which is the form the spike proved
// the app animates.
func diffTransforms(prev, next map[string]*commonpb.Transform) []worldstatestore.TransformChange {
	var out []worldstatestore.TransformChange

	for name, n := range next {
		p, existed := prev[name]
		if !existed {
			out = append(out, worldstatestore.TransformChange{
				ChangeType: pb.TransformChangeType_TRANSFORM_CHANGE_TYPE_ADDED,
				Transform:  n,
			})
			continue
		}
		if paths := changedPosePaths(p.PoseInObserverFrame.Pose, n.PoseInObserverFrame.Pose); len(paths) > 0 {
			out = append(out, worldstatestore.TransformChange{
				ChangeType: pb.TransformChangeType_TRANSFORM_CHANGE_TYPE_UPDATED,
				Transform: &commonpb.Transform{
					Uuid: n.Uuid,
					PoseInObserverFrame: &commonpb.PoseInFrame{
						Pose: n.PoseInObserverFrame.Pose,
					},
				},
				UpdatedFields: paths,
			})
		}
	}

	for name, p := range prev {
		if _, still := next[name]; !still {
			out = append(out, worldstatestore.TransformChange{
				ChangeType: pb.TransformChangeType_TRANSFORM_CHANGE_TYPE_REMOVED,
				Transform:  p,
			})
		}
	}
	return out
}

// changedPosePaths lists the field-mask paths whose values differ. Geometry and
// parentage never change for a given frame, so only the pose is compared.
func changedPosePaths(a, b *commonpb.Pose) []string {
	var paths []string
	for _, f := range []struct {
		name string
		a, b float64
	}{
		{"x", a.X, b.X}, {"y", a.Y, b.Y}, {"z", a.Z, b.Z},
		{"oX", a.OX, b.OX}, {"oY", a.OY, b.OY}, {"oZ", a.OZ, b.OZ},
		{"theta", a.Theta, b.Theta},
	} {
		if f.a != f.b {
			paths = append(paths, fmt.Sprintf("poseInObserverFrame.pose.%s", f.name))
		}
	}
	return paths
}

// applyCommitted advances the delivered-state map by only the changes that were
// actually enqueued.
//
// This is what makes drop-on-full safe. Dropping is harmless for UPDATED — the
// next diff regenerates against the current value — but ADDED and REMOVED are
// terminal: because diffTransforms compares consecutive maps, a dropped REMOVED
// would leave the frame absent from both prev and next on every later tick and so
// never be re-emitted, stranding the commanded box in the scene forever after the
// operator releases the grip. Advancing conditionally makes a dropped change
// pending rather than lost, and it self-heals on the next tick.
//
// It takes next as well as prev because an UPDATED change carries only a partial
// transform — storing c.Transform directly would corrupt the delivered-state map
// with a payload that has no geometry or parent frame. The full value is looked
// up in next instead.
func applyCommitted(
	prev, next map[string]*commonpb.Transform,
	changes []worldstatestore.TransformChange,
	sent []bool,
) map[string]*commonpb.Transform {
	out := make(map[string]*commonpb.Transform, len(prev))
	for k, v := range prev {
		out[k] = v
	}
	for i, c := range changes {
		if i >= len(sent) || !sent[i] {
			continue
		}
		name := string(c.Transform.Uuid)
		switch c.ChangeType {
		case pb.TransformChangeType_TRANSFORM_CHANGE_TYPE_REMOVED:
			delete(out, name)
		default:
			// ADDED and UPDATED both mean "the consumer now has the current value".
			if full, ok := next[name]; ok {
				out[name] = full
			}
		}
	}
	return out
}
