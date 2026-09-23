package vive

import (
	"strings"
	"testing"
)

// Hand names are load-bearing identifiers, so Validate must reject configs where
// two hands would collide on one key. The visualizer derives frame names from
// them, so two unnamed hands both produce "vive--controller" and only the last
// renders — indistinguishable from tracking having died. (The capture-ownership
// branch relies on the same guarantee for its owner set.)
func TestTeleopConfig_ValidateHandNames(t *testing.T) {
	hand := func(name string) HandConfig {
		return HandConfig{Name: name, Controller: "ctrl-" + name, Arm: "arm-" + name}
	}

	tests := []struct {
		name    string
		hands   []HandConfig
		wantErr string // substring; empty means the config must validate
	}{
		{
			name:  "distinct names accepted",
			hands: []HandConfig{hand("left"), hand("right")},
		},
		{
			name:  "single named hand accepted",
			hands: []HandConfig{hand("left")},
		},
		{
			name:    "unnamed hand rejected",
			hands:   []HandConfig{{Controller: "ctrl", Arm: "arm"}},
			wantErr: "every hand must have a name",
		},
		{
			// Both hands would map onto the same anonymous owner key.
			name:    "two unnamed hands rejected",
			hands:   []HandConfig{{Controller: "c1", Arm: "a1"}, {Controller: "c2", Arm: "a2"}},
			wantErr: "every hand must have a name",
		},
		{
			name:    "duplicate names rejected",
			hands:   []HandConfig{hand("hand"), hand("hand")},
			wantErr: `duplicate hand name "hand"`,
		},
	}

	for _, tc := range tests {
		t.Run(tc.name, func(t *testing.T) {
			cfg := &TeleopConfig{Hands: tc.hands}
			deps, _, err := cfg.Validate("path")

			if tc.wantErr == "" {
				if err != nil {
					t.Fatalf("Validate() = %v, want nil", err)
				}
				if len(deps) == 0 {
					t.Error("Validate() returned no dependencies for a valid config")
				}
				return
			}

			if err == nil {
				t.Fatalf("Validate() = nil, want an error containing %q", tc.wantErr)
			}
			if !strings.Contains(err.Error(), tc.wantErr) {
				t.Errorf("Validate() = %q, want it to contain %q", err.Error(), tc.wantErr)
			}
		})
	}
}
