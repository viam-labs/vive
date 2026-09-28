package vive

import "testing"

func TestControllerConfigValidateSkipsTeleopActions(t *testing.T) {
	cfg := &ControllerConfig{
		MenuAction: &ButtonAction{Component: "xarm-home", Method: "set_position", Args: []interface{}{2.0}},
		TrackpadDownAction: &ButtonAction{
			Component: "vive-teleop", Method: "do_command",
			Args: []interface{}{map[string]interface{}{"toggle_rotation_mode": true}},
		},
		TrackpadLeftAction: &ButtonAction{
			Component: "vive-teleop", Method: "do_command",
			Args: []interface{}{map[string]interface{}{"adjust_calibration": -5.0}},
		},
		TrackpadUpAction: &ButtonAction{
			Component: "some-sensor", Method: "do_command",
			Args: []interface{}{map[string]interface{}{"status": true}},
		},
	}
	required, optional, err := cfg.Validate("test")
	if err != nil {
		t.Fatal(err)
	}
	if len(required) != 0 {
		t.Fatalf("expected no required deps, got %v", required)
	}
	want := []string{"xarm-home", "some-sensor"}
	if len(optional) != len(want) {
		t.Fatalf("optional deps = %v, want %v", optional, want)
	}
	for i := range want {
		if optional[i] != want[i] {
			t.Fatalf("optional deps = %v, want %v", optional, want)
		}
	}
}
