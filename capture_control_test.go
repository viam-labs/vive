package vive

import (
	"context"
	"testing"
	"time"

	"go.viam.com/rdk/components/sensor"
	"go.viam.com/rdk/logging"
)

// seqCall records one sequenceFn invocation.
type seqCall struct {
	start, end time.Time
	tags       []string
}

// seqRecorder stands in for createSequence so tests can observe session windows
// without dialing app.viam.com. Buffered so a recorded call never blocks the
// goroutine the production code spawns.
type seqRecorder struct {
	ch chan seqCall
}

func newSeqRecorder() *seqRecorder {
	return &seqRecorder{ch: make(chan seqCall, 8)}
}

func (r *seqRecorder) fn(start, end time.Time, tags []string) {
	r.ch <- seqCall{start: start, end: end, tags: tags}
}

// next waits for one recorded call, failing if none arrives.
func (r *seqRecorder) next(t *testing.T) seqCall {
	t.Helper()
	select {
	case c := <-r.ch:
		return c
	case <-time.After(2 * time.Second):
		t.Fatal("expected a sequence to be created, got none")
		return seqCall{}
	}
}

// none asserts no call arrives. The wait is short because the production call is
// spawned immediately; this is a negative assertion, not a synchronization point.
func (r *seqRecorder) none(t *testing.T) {
	t.Helper()
	select {
	case c := <-r.ch:
		t.Fatalf("expected no sequence, got one with tags=%v", c.tags)
	case <-time.After(150 * time.Millisecond):
	}
}

// newTestCC builds a captureControl directly, bypassing newCaptureControl's
// resource.Config plumbing, and wires in a recording sequenceFn.
func newTestCC(t *testing.T, cfg *CaptureControlConfig) (*captureControl, *seqRecorder) {
	t.Helper()
	if cfg == nil {
		cfg = &CaptureControlConfig{ArmName: "arm"}
	}
	rec := newSeqRecorder()
	cc := &captureControl{
		name:          sensor.Named("capture"),
		logger:        logging.NewTestLogger(t),
		cfg:           cfg,
		captureFreqHz: 10,
		activeHands:   make(map[string]struct{}),
	}
	cc.sequenceFn = rec.fn
	return cc, rec
}

// do issues a DoCommand and fails on error.
func do(t *testing.T, cc *captureControl, cmd map[string]interface{}) map[string]interface{} {
	t.Helper()
	resp, err := cc.DoCommand(context.Background(), cmd)
	if err != nil {
		t.Fatalf("DoCommand(%v): %v", cmd, err)
	}
	return resp
}

func TestCaptureControl_SingleGripCreatesOneSequence(t *testing.T) {
	cc, rec := newTestCC(t, nil)

	do(t, cc, map[string]interface{}{"start-capture": "left"})
	if !cc.capturing {
		t.Fatal("expected capturing after start")
	}
	start := cc.sessionStart
	tags := append([]string{}, cc.sessionTags...)

	do(t, cc, map[string]interface{}{"stop-capture": "left"})
	if cc.capturing {
		t.Fatal("expected not capturing after stop")
	}

	got := rec.next(t)
	if !got.start.Equal(start) {
		t.Errorf("sequence start = %v, want %v", got.start, start)
	}
	if got.end.Before(start) {
		t.Errorf("sequence end %v precedes start %v", got.end, start)
	}
	if len(got.tags) != len(tags) || got.tags[0] != tags[0] {
		t.Errorf("sequence tags = %v, want %v", got.tags, tags)
	}
}

// hands reads the current owner set via the status command.
func hands(t *testing.T, cc *captureControl) []string {
	t.Helper()
	resp := do(t, cc, map[string]interface{}{"status": true})
	got, ok := resp["active_hands"].([]string)
	if !ok {
		t.Fatalf("status active_hands = %v (%T), want []string", resp["active_hands"], resp["active_hands"])
	}
	return got
}

func equalStrings(a, b []string) bool {
	if len(a) != len(b) {
		return false
	}
	for i := range a {
		if a[i] != b[i] {
			return false
		}
	}
	return true
}

func TestCaptureControl_OwnerSet(t *testing.T) {
	tests := []struct {
		name          string
		ops           []map[string]interface{}
		wantCapturing bool
		wantHands     []string
		wantSequences int
	}{
		{
			// Two hands share one session; the first release must not end it.
			name: "two-hand overlap keeps session open until last release",
			ops: []map[string]interface{}{
				{"start-capture": "left"},
				{"start-capture": "right"},
				{"stop-capture": "left"},
			},
			wantCapturing: true,
			wantHands:     []string{"right"},
			wantSequences: 0,
		},
		{
			name: "both hands released closes the session once",
			ops: []map[string]interface{}{
				{"start-capture": "left"},
				{"start-capture": "right"},
				{"stop-capture": "left"},
				{"stop-capture": "right"},
			},
			wantCapturing: false,
			wantHands:     []string{},
			wantSequences: 1,
		},
		{
			// Duplicate start must not need two stops to balance.
			name: "duplicate start from one hand is idempotent",
			ops: []map[string]interface{}{
				{"start-capture": "left"},
				{"start-capture": "left"},
				{"stop-capture": "left"},
			},
			wantCapturing: false,
			wantHands:     []string{},
			wantSequences: 1,
		},
		{
			// Hand B's start failed, so its stop must not end hand A's session.
			name: "stop from a hand that never started leaves the holder alone",
			ops: []map[string]interface{}{
				{"start-capture": "left"},
				{"stop-capture": "right"},
			},
			wantCapturing: true,
			wantHands:     []string{"left"},
			wantSequences: 0,
		},
		{
			name: "duplicate stop is idempotent",
			ops: []map[string]interface{}{
				{"start-capture": "left"},
				{"stop-capture": "left"},
				{"stop-capture": "left"},
			},
			wantCapturing: false,
			wantHands:     []string{},
			wantSequences: 1,
		},
		{
			name: "stop with no session open is a no-op",
			ops: []map[string]interface{}{
				{"stop-capture": "right"},
			},
			wantCapturing: false,
			wantHands:     []string{},
			wantSequences: 0,
		},
		{
			// The legacy {"start-capture": true} form collapses onto one anonymous owner.
			name: "legacy bool payload round-trips",
			ops: []map[string]interface{}{
				{"start-capture": true},
				{"stop-capture": true},
			},
			wantCapturing: false,
			wantHands:     []string{},
			wantSequences: 1,
		},
	}

	for _, tc := range tests {
		t.Run(tc.name, func(t *testing.T) {
			cc, rec := newTestCC(t, nil)
			for _, op := range tc.ops {
				do(t, cc, op)
			}

			if cc.capturing != tc.wantCapturing {
				t.Errorf("capturing = %v, want %v", cc.capturing, tc.wantCapturing)
			}
			if got := hands(t, cc); !equalStrings(got, tc.wantHands) {
				t.Errorf("active_hands = %v, want %v", got, tc.wantHands)
			}
			for i := 0; i < tc.wantSequences; i++ {
				rec.next(t)
			}
			rec.none(t)
		})
	}
}

func TestCaptureControl_ConsecutiveSessionsAreIndependent(t *testing.T) {
	cc, rec := newTestCC(t, nil)

	do(t, cc, map[string]interface{}{"start-capture": "left"})
	firstTag := cc.sessionTags[0]
	do(t, cc, map[string]interface{}{"stop-capture": "left"})
	first := rec.next(t)

	// The session tag has one-second resolution, so force a distinct stamp.
	time.Sleep(1100 * time.Millisecond)

	do(t, cc, map[string]interface{}{"start-capture": "left"})
	if !cc.capturing {
		t.Fatal("expected a second session to open")
	}
	if cc.sessionTags[0] == firstTag {
		t.Errorf("second session reused the first session's tag %q", firstTag)
	}
	do(t, cc, map[string]interface{}{"stop-capture": "left"})
	second := rec.next(t)

	// The second window must not reach back into the first session.
	if !second.start.After(first.end) {
		t.Errorf("second session start %v is not after first session end %v",
			second.start, first.end)
	}
	if second.tags[0] == first.tags[0] {
		t.Errorf("both sequences carry tag %q", first.tags[0])
	}
}

func TestCaptureControl_SessionEndClearsOwners(t *testing.T) {
	cc, _ := newTestCC(t, nil)

	// Arrange a leaked owner alongside a real one.
	cc.activeHands["ghost"] = struct{}{}
	do(t, cc, map[string]interface{}{"start-capture": "left"})

	// Releasing "left" leaves "ghost" in the set, so the session correctly stays
	// open — that is the guarantee that one hand cannot end another's session.
	do(t, cc, map[string]interface{}{"stop-capture": "left"})
	if !cc.capturing {
		t.Fatal("session ended while an owner was still held")
	}

	// Releasing the last owner ends the session, and ending it clears the whole
	// set — so no leaked owner can survive to wedge the next session.
	do(t, cc, map[string]interface{}{"stop-capture": "ghost"})
	if cc.capturing {
		t.Error("expected not capturing after the last owner released")
	}
	if got := hands(t, cc); len(got) != 0 {
		t.Errorf("active_hands = %v after session end, want empty", got)
	}
}
