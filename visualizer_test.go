package vive

import (
	"math"
	"slices"
	"sort"
	"strings"
	"testing"
	"time"

	"github.com/go-gl/mathgl/mgl64"
	commonpb "go.viam.com/api/common/v1"
	pb "go.viam.com/api/service/worldstatestore/v1"
	"go.viam.com/rdk/services/worldstatestore"
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

func changeFor(changes []worldstatestore.TransformChange, uuid string) (worldstatestore.TransformChange, bool) {
	for _, c := range changes {
		if string(c.Transform.Uuid) == uuid {
			return c, true
		}
	}
	return worldstatestore.TransformChange{}, false
}

func TestDiffTransforms(t *testing.T) {
	now := time.Unix(1000, 0)
	base := VisualState{ComposedCalib: mgl64.Ident4(), Hands: []HandVisual{liveHand("left", now)}}
	idle := buildTransforms(base, testCfg(), now)

	// Clone: VisualState.Hands is a slice, so assigning the struct aliases the
	// backing array and later fixtures would mutate `base` out from under the
	// earlier subtests.
	gripping := base
	gripping.Hands = slices.Clone(base.Hands)
	gripping.Hands[0].Controlling = true
	gripping.Hands[0].Commanded = &Pose{X: 10}
	gripped := buildTransforms(gripping, testCfg(), now)

	t.Run("first build adds everything", func(t *testing.T) {
		changes := diffTransforms(nil, idle)
		if len(changes) != len(idle) {
			t.Fatalf("got %d changes, want %d", len(changes), len(idle))
		}
		for _, c := range changes {
			if c.ChangeType != pb.TransformChangeType_TRANSFORM_CHANGE_TYPE_ADDED {
				t.Errorf("%s: type = %v, want ADDED", c.Transform.Uuid, c.ChangeType)
			}
			// ADDED carries the full transform, so the app can create the object.
			if c.Transform.ReferenceFrame == "" || c.Transform.PhysicalObject == nil {
				t.Errorf("%s: ADDED must carry a full transform", c.Transform.Uuid)
			}
			if len(c.UpdatedFields) != 0 {
				t.Errorf("%s: ADDED must not carry a field mask", c.Transform.Uuid)
			}
		}
	})

	t.Run("no change produces nothing", func(t *testing.T) {
		if changes := diffTransforms(idle, idle); len(changes) != 0 {
			t.Errorf("idle module is not silent: %d changes", len(changes))
		}
	})

	t.Run("grip adds the commanded box", func(t *testing.T) {
		changes := diffTransforms(idle, gripped)
		c, ok := changeFor(changes, "vive-left-commanded")
		if !ok {
			t.Fatal("no change for the commanded box")
		}
		if c.ChangeType != pb.TransformChangeType_TRANSFORM_CHANGE_TYPE_ADDED {
			t.Errorf("type = %v, want ADDED", c.ChangeType)
		}
	})

	t.Run("release removes the commanded box", func(t *testing.T) {
		changes := diffTransforms(gripped, idle)
		c, ok := changeFor(changes, "vive-left-commanded")
		if !ok {
			t.Fatal("no change for the commanded box")
		}
		if c.ChangeType != pb.TransformChangeType_TRANSFORM_CHANGE_TYPE_REMOVED {
			t.Errorf("type = %v, want REMOVED", c.ChangeType)
		}
	})

	t.Run("moved pose produces a partial UPDATED", func(t *testing.T) {
		moved := base
		moved.Hands = slices.Clone(base.Hands)
		moved.Hands[0].ControllerPos = [3]float64{9, 2, 3} // X only
		next := buildTransforms(moved, testCfg(), now)

		changes := diffTransforms(idle, next)
		c, ok := changeFor(changes, "vive-left-controller")
		if !ok {
			t.Fatal("no change for the controller")
		}
		if c.ChangeType != pb.TransformChangeType_TRANSFORM_CHANGE_TYPE_UPDATED {
			t.Fatalf("type = %v, want UPDATED", c.ChangeType)
		}
		// Partial payload, matching the RDK fake: UUID plus changed fields only.
		if c.Transform.PhysicalObject != nil {
			t.Error("UPDATED must not resend the geometry")
		}
		if len(c.UpdatedFields) == 0 {
			t.Fatal("UPDATED must carry a field mask")
		}
		// lowerCamelCase dotted paths, matching the fake — the only form the spike
		// proved the app animates.
		for _, p := range c.UpdatedFields {
			if !strings.HasPrefix(p, "poseInObserverFrame.") {
				t.Errorf("field path %q is not a lowerCamelCase dotted path", p)
			}
		}
	})
}

func allSent(n int) []bool {
	s := make([]bool, n)
	for i := range s {
		s[i] = true
	}
	return s
}

func TestApplyCommitted_DroppedTerminalChangeIsRetried(t *testing.T) {
	now := time.Unix(1000, 0)
	gripping := VisualState{ComposedCalib: mgl64.Ident4(), Hands: []HandVisual{liveHand("left", now)}}
	gripping.Hands[0].Controlling = true
	gripping.Hands[0].Commanded = &Pose{X: 10}

	gripped := buildTransforms(gripping, testCfg(), now)
	idle := buildTransforms(
		VisualState{ComposedCalib: mgl64.Ident4(), Hands: []HandVisual{liveHand("left", now)}},
		testCfg(), now,
	)

	// Deliver everything so prev == gripped.
	prev := applyCommitted(nil, gripped, diffTransforms(nil, gripped), allSent(len(gripped)))

	// The operator releases, but the REMOVED is dropped because the consumer is slow.
	changes := diffTransforms(prev, idle)
	c, ok := changeFor(changes, "vive-left-commanded")
	if !ok || c.ChangeType != pb.TransformChangeType_TRANSFORM_CHANGE_TYPE_REMOVED {
		t.Fatal("expected a REMOVED for the commanded box")
	}
	sent := make([]bool, len(changes))
	for i, ch := range changes {
		sent[i] = string(ch.Transform.Uuid) != "vive-left-commanded" // this one dropped
	}
	prev = applyCommitted(prev, idle, changes, sent)

	// Because prev did not advance for the dropped change, the next tick re-emits it.
	// Advancing prev wholesale would leave the box in the app's scene forever.
	again := diffTransforms(prev, idle)
	c2, ok := changeFor(again, "vive-left-commanded")
	if !ok {
		t.Fatal("dropped REMOVED was never re-emitted — geometry is stranded")
	}
	if c2.ChangeType != pb.TransformChangeType_TRANSFORM_CHANGE_TYPE_REMOVED {
		t.Errorf("re-emitted type = %v, want REMOVED", c2.ChangeType)
	}
}

func TestApplyCommitted_DeliveredChangesAdvance(t *testing.T) {
	now := time.Unix(1000, 0)
	idle := buildTransforms(
		VisualState{ComposedCalib: mgl64.Ident4(), Hands: []HandVisual{liveHand("left", now)}},
		testCfg(), now,
	)

	prev := applyCommitted(nil, idle, diffTransforms(nil, idle), allSent(len(idle)))
	if changes := diffTransforms(prev, idle); len(changes) != 0 {
		t.Errorf("fully delivered state still reports %d changes", len(changes))
	}
}
