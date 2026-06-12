package vive

import (
	"math"
	"testing"
)

const medEps = 1e-9

// pushAll feeds a sequence through the filter and returns the per-sample outputs.
func pushAll(f *MedianFilter, in []float64) []float64 {
	out := make([]float64, len(in))
	for i, v := range in {
		out[i] = f.Push(v)
	}
	return out
}

func seqEqual(t *testing.T, name string, got, want []float64) {
	t.Helper()
	if len(got) != len(want) {
		t.Fatalf("%s: length %d != %d", name, len(got), len(want))
	}
	for i := range want {
		if math.Abs(got[i]-want[i]) > medEps {
			t.Errorf("%s: at %d got %v, want %v\n got=%v\nwant=%v", name, i, got[i], want[i], got, want)
		}
	}
}

func TestMedianFilter_SequenceBehavior(t *testing.T) {
	tests := []struct {
		name   string
		window int
		in     []float64
		want   []float64 // expected per-sample output
	}{
		{
			// Single-frame full-scale spike in steady state is fully rejected.
			// This is the bug: a lone 0->1->0 trigger glitch must not reach the gripper.
			name:   "single-frame spike rejected (w3)",
			window: 3,
			in:     []float64{0, 0, 0, 1, 0, 0},
			want:   []float64{0, 0, 0, 0, 0, 0},
		},
		{
			// Two-frame spike is rejected by a window of 5 (2 <= floor(5/2)).
			name:   "two-frame spike rejected (w5)",
			window: 5,
			in:     []float64{0, 0, 0, 0, 0, 1, 1, 0, 0, 0},
			want:   []float64{0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
		},
		{
			// A three-frame spike exceeds floor(5/2)=2 and leaks through — documents the bound.
			name:   "three-frame spike leaks (w5)",
			window: 5,
			in:     []float64{0, 0, 0, 0, 0, 1, 1, 1, 0, 0},
			want:   []float64{0, 0, 0, 0, 0, 0, 0, 1, 1, 1},
		},
		{
			// A genuine proportional squeeze (gradual ramp) is preserved, not falsely zeroed.
			// Median lags ~1 frame but tracks monotonically upward.
			name:   "real squeeze preserved (w3)",
			window: 3,
			in:     []float64{0, 0, 0.2, 0.4, 0.6, 0.8},
			want:   []float64{0, 0, 0, 0.2, 0.4, 0.6},
		},
		{
			// Window 1 is a pass-through: every value (including a spike) is returned as-is.
			name:   "window 1 passthrough",
			window: 1,
			in:     []float64{0, 1, 0, 0.5},
			want:   []float64{0, 1, 0, 0.5},
		},
	}

	for _, tc := range tests {
		t.Run(tc.name, func(t *testing.T) {
			f := NewMedianFilter(tc.window)
			seqEqual(t, tc.name, pushAll(f, tc.in), tc.want)
		})
	}
}

func TestMedianFilter_WindowForcedOddAndPositive(t *testing.T) {
	cases := []struct{ in, want int }{
		{-3, 1}, {0, 1}, {1, 1}, {2, 3}, {3, 3}, {4, 5}, {5, 5},
	}
	for _, c := range cases {
		if got := NewMedianFilter(c.in).window; got != c.want {
			t.Errorf("NewMedianFilter(%d).window = %d, want %d", c.in, got, c.want)
		}
	}
}

func TestMedianFilter_Reset(t *testing.T) {
	f := NewMedianFilter(3)
	// Prime steady state at 0, then inject a spike that is currently outvoted.
	pushAll(f, []float64{0, 0, 0, 1})
	f.Reset()
	// After reset the filter is in warm-up again: the next lone sample passes through.
	if got := f.Push(0.42); math.Abs(got-0.42) > medEps {
		t.Errorf("after Reset, first Push = %v, want 0.42 (warm-up passthrough)", got)
	}
}

func TestMedianFilter_NilSafe(t *testing.T) {
	var f *MedianFilter
	if got := f.Push(0.7); math.Abs(got-0.7) > medEps {
		t.Errorf("nil filter Push = %v, want 0.7 passthrough", got)
	}
	f.Reset() // must not panic
}
