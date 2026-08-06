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
