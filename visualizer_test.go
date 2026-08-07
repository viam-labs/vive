package vive

import (
	"math"
	"sort"
	"testing"
	"time"

	"github.com/go-gl/mathgl/mgl64"
	commonpb "go.viam.com/api/common/v1"
)

func testCfg() VisualizerConfig {
	return VisualizerConfig{
		TeleopService: "teleop",
		PublishRateHz: 20,
		StaleAfterMS:  500,
	}
}

// liveHand returns a hand snapshot that is connected and fresh as of now.
func liveHand(name string, now time.Time) HandVisual {
	return HandVisual{
		Name:          name,
		PublishedAt:   now,
		Connected:     true,
		ControllerPos: [3]float64{1, 2, 3}, // metres
		ControllerMat: mgl64.Ident4(),
	}
}

func keysOf(m map[string]*commonpb.Transform) []string {
	out := make([]string, 0, len(m))
	for k := range m {
		out = append(out, k)
	}
	sort.Strings(out)
	return out
}

func poseNearlyEqual(a, b *commonpb.Pose) bool {
	const eps = 1e-9
	return math.Abs(a.X-b.X) < eps && math.Abs(a.Y-b.Y) < eps && math.Abs(a.Z-b.Z) < eps &&
		math.Abs(a.OX-b.OX) < eps && math.Abs(a.OY-b.OY) < eps && math.Abs(a.OZ-b.OZ) < eps &&
		math.Abs(a.Theta-b.Theta) < eps
}

func TestBuildTransforms_FramesAndParents(t *testing.T) {
	now := time.Unix(1000, 0)
	st := VisualState{
		ComposedCalib: mgl64.Ident4(),
		Hands:         []HandVisual{liveHand("left", now)},
	}

	got := buildTransforms(st, testCfg(), now)

	lh, ok := got["vive-lighthouse"]
	if !ok {
		t.Fatalf("missing lighthouse frame; got %v", keysOf(got))
	}
	if lh.PoseInObserverFrame.ReferenceFrame != "world" {
		t.Errorf("lighthouse parent = %q, want world", lh.PoseInObserverFrame.ReferenceFrame)
	}

	ctrl, ok := got["vive-left-controller"]
	if !ok {
		t.Fatalf("missing controller frame; got %v", keysOf(got))
	}
	// The parent chain must reach world through the lighthouse, so the app
	// composes the calibration itself and a bad calibration is visible.
	if ctrl.PoseInObserverFrame.ReferenceFrame != "vive-lighthouse" {
		t.Errorf("controller parent = %q, want vive-lighthouse", ctrl.PoseInObserverFrame.ReferenceFrame)
	}
	if ctrl.ReferenceFrame != "vive-left-controller" {
		t.Errorf("controller own frame = %q", ctrl.ReferenceFrame)
	}
	// Name must also be on the geometry label: the app surfaces names, but which
	// field drives that is unconfirmed, so both are set.
	if ctrl.PhysicalObject.Label != "vive-left-controller" {
		t.Errorf("controller label = %q", ctrl.PhysicalObject.Label)
	}
}

func TestBuildTransforms_MetresToMillimetres(t *testing.T) {
	now := time.Unix(1000, 0)
	st := VisualState{
		ComposedCalib: mgl64.Ident4(),
		Hands:         []HandVisual{liveHand("left", now)},
	}

	got := buildTransforms(st, testCfg(), now)
	p := got["vive-left-controller"].PoseInObserverFrame.Pose

	// 1,2,3 metres -> 1000,2000,3000 mm. Note scale is NOT applied: the operator
	// must see their hand where it physically is, whatever amplification teleop uses.
	if p.X != 1000 || p.Y != 2000 || p.Z != 3000 {
		t.Errorf("pose = (%v,%v,%v), want (1000,2000,3000)", p.X, p.Y, p.Z)
	}
}

func TestBuildTransforms_YawAppliedOnce(t *testing.T) {
	now := time.Unix(1000, 0)
	const yaw = 0.4

	st := VisualState{
		// Already composed, as VisualState() produces it.
		ComposedCalib: mgl64.HomogRotate3DZ(yaw),
		CalibYaw:      yaw,
	}

	got := buildTransforms(st, testCfg(), now)
	p := got["vive-lighthouse"].PoseInObserverFrame.Pose

	// A frame rotated by 2*yaw is the failure this guards. Compare against the
	// same single rotation rendered with CalibYaw zeroed: if the implementation
	// ignores CalibYaw as it must, the two are identical.
	want := buildTransforms(
		VisualState{ComposedCalib: mgl64.HomogRotate3DZ(yaw), CalibYaw: 0},
		testCfg(), now,
	)["vive-lighthouse"].PoseInObserverFrame.Pose

	if !poseNearlyEqual(p, want) {
		t.Errorf("yaw applied twice: got %+v, want %+v", p, want)
	}
}

func TestBuildTransforms_LighthouseOffsetOnly(t *testing.T) {
	now := time.Unix(1000, 0)
	cfg := testCfg()
	cfg.LighthouseOffsetMM = [3]float64{100, 200, 300}

	st := VisualState{
		ComposedCalib: mgl64.Ident4(),
		Hands:         []HandVisual{liveHand("left", now)},
	}
	got := buildTransforms(st, cfg, now)

	lh := got["vive-lighthouse"].PoseInObserverFrame.Pose
	if lh.X != 100 || lh.Y != 200 || lh.Z != 300 {
		t.Errorf("lighthouse pose = (%v,%v,%v), want the offset", lh.X, lh.Y, lh.Z)
	}
	// The offset is cosmetic and applies to the lighthouse frame alone; the
	// controller is a child, so the app composes it and we must not add it twice.
	ctrl := got["vive-left-controller"].PoseInObserverFrame.Pose
	if ctrl.X != 1000 {
		t.Errorf("controller X = %v, want 1000 (offset must not be applied here)", ctrl.X)
	}
}

func TestBuildTransforms_Presence(t *testing.T) {
	now := time.Unix(1000, 0)

	tests := []struct {
		name        string
		hand        HandVisual
		wantCtrl    bool
		wantCommand bool
	}{
		{
			name:     "connected and idle shows only the controller",
			hand:     liveHand("left", now),
			wantCtrl: true,
		},
		{
			name: "gripping adds the commanded box",
			hand: func() HandVisual {
				h := liveHand("left", now)
				h.Controlling = true
				h.Commanded = &Pose{X: 10, Y: 20, Z: 30}
				return h
			}(),
			wantCtrl:    true,
			wantCommand: true,
		},
		{
			name: "disconnected removes everything",
			hand: func() HandVisual {
				h := liveHand("left", now)
				h.Connected = false
				return h
			}(),
		},
		{
			// Covers the surviveMu skip, which has no publish point of its own.
			name: "stale snapshot removes everything",
			hand: liveHand("left", now.Add(-2*time.Second)),
		},
		{
			name: "controlling without a commanded pose shows no box",
			hand: func() HandVisual {
				h := liveHand("left", now)
				h.Controlling = true
				return h
			}(),
			wantCtrl: true,
		},
	}

	for _, tc := range tests {
		t.Run(tc.name, func(t *testing.T) {
			got := buildTransforms(
				VisualState{ComposedCalib: mgl64.Ident4(), Hands: []HandVisual{tc.hand}},
				testCfg(), now,
			)
			_, haveCtrl := got["vive-left-controller"]
			_, haveCmd := got["vive-left-commanded"]

			if haveCtrl != tc.wantCtrl {
				t.Errorf("controller present = %v, want %v", haveCtrl, tc.wantCtrl)
			}
			if haveCmd != tc.wantCommand {
				t.Errorf("commanded present = %v, want %v", haveCmd, tc.wantCommand)
			}
			// The lighthouse frame is unconditional — it describes calibration, not a hand.
			if _, ok := got["vive-lighthouse"]; !ok {
				t.Error("lighthouse frame must always be present")
			}
		})
	}
}

func TestBuildTransforms_UUIDsAreStableAndNameDerived(t *testing.T) {
	now := time.Unix(1000, 0)
	st := VisualState{
		ComposedCalib: mgl64.Ident4(),
		Hands:         []HandVisual{liveHand("left", now)},
	}

	a := buildTransforms(st, testCfg(), now)
	b := buildTransforms(st, testCfg(), now.Add(time.Millisecond))

	for name, ta := range a {
		tb, ok := b[name]
		if !ok {
			t.Fatalf("frame %q vanished between builds", name)
		}
		// A UUID regenerated per build makes the app see a new object every tick.
		if string(ta.Uuid) != string(tb.Uuid) {
			t.Errorf("%s uuid unstable: %q vs %q", name, ta.Uuid, tb.Uuid)
		}
		if string(ta.Uuid) != name {
			t.Errorf("%s uuid = %q, want the frame name", name, ta.Uuid)
		}
	}
}

func TestBuildTransforms_LighthouseBoxLegibleWithoutColour(t *testing.T) {
	now := time.Unix(1000, 0)
	got := buildTransforms(VisualState{ComposedCalib: mgl64.Ident4()}, testCfg(), now)

	box := got["vive-lighthouse"].PhysicalObject.GetBox()
	if box == nil {
		t.Fatal("lighthouse geometry is not a box")
	}
	d := box.DimsMm
	// The spike proved colour is ignored, so orientation must be readable from
	// dimensions alone: all three extents distinguishable, X longest, Z shortest.
	if !(d.X > d.Y && d.Y > d.Z) {
		t.Errorf("lighthouse dims (%v,%v,%v) are not strictly ordered X>Y>Z", d.X, d.Y, d.Z)
	}
}
