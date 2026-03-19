package main

import (
	"context"
	"flag"
	"fmt"
	"os"
	"os/signal"
	"path/filepath"
	"syscall"
	"time"

	"go.viam.com/rdk/components/arm"
	"go.viam.com/rdk/components/gripper"
	input "go.viam.com/rdk/components/input"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/resource"
	"go.viam.com/rdk/robot/client"
	"go.viam.com/rdk/services/generic"
	"go.viam.com/rdk/services/motion"
	"go.viam.com/utils/rpc"

	vive "vive"
)

func main() {
	err := realMain()
	if err != nil {
		fmt.Fprintf(os.Stderr, "[teleop] %v\n", err)
		os.Exit(1)
	}
}

func realMain() error {
	hz := flag.Int("hz", 90, "Polling rate in Hz")
	address := flag.String("address", "", "Viam machine address")
	keyID := flag.String("key-id", "", "Viam API key ID")
	key := flag.String("key", "", "Viam API key")
	leftArm := flag.String("left-arm", "right-arm", "Arm controlled by the left controller")
	rightArm := flag.String("right-arm", "left-arm", "Arm controlled by the right controller")
	leftGripper := flag.String("left-gripper", "right-gripper", "Gripper controlled by the left controller")
	rightGripper := flag.String("right-gripper", "left-gripper", "Gripper controlled by the right controller")
	scale := flag.Float64("scale", 1.0, "Position scale factor")
	rotEnabled := flag.Bool("rotation", true, "Enable orientation tracking")
	posDeadzone := flag.Float64("pos-deadzone", 0.5, "Position dead-zone in mm")
	rotDeadzone := flag.Float64("rot-deadzone", 1.0, "Rotation dead-zone in degrees")
	smoothAlpha := flag.Float64("smooth-alpha", 0.5, "EMA smoothing alpha")
	flag.Parse()

	if *address == "" {
		return fmt.Errorf("--address is required")
	}

	ctx, cancel := context.WithCancel(context.Background())
	defer cancel()

	logger := logging.NewLogger("teleop")
	logger.Infof("Connecting to %s ...", *address)

	dialOpts := []rpc.DialOption{}
	if *keyID != "" && *key != "" {
		dialOpts = append(dialOpts, rpc.WithEntityCredentials(*keyID, rpc.Credentials{
			Type:    rpc.CredentialsTypeAPIKey,
			Payload: *key,
		}))
	}

	robot, err := client.New(ctx, *address, logger, client.WithDialOptions(dialOpts...))
	if err != nil {
		return fmt.Errorf("failed to connect to robot: %w", err)
	}
	defer robot.Close(ctx)
	logger.Info("Connected to robot")

	exePath, err := os.Executable()
	if err != nil {
		return fmt.Errorf("cannot determine executable path: %w", err)
	}
	calibDir := filepath.Dir(exePath)

	// Create vive-controller instances for left and right.
	leftCtrl, err := vive.NewViveController(ctx, nil, input.Named("left-vive"), &vive.ControllerConfig{}, logger)
	if err != nil {
		return fmt.Errorf("left controller: %w", err)
	}
	defer leftCtrl.Close(ctx)

	rightCtrl, err := vive.NewViveController(ctx, nil, input.Named("right-vive"), &vive.ControllerConfig{}, logger)
	if err != nil {
		return fmt.Errorf("right controller: %w", err)
	}
	defer rightCtrl.Close(ctx)

	// Resolve arm/gripper/motion from robot client.
	leftArmRes, err := arm.FromRobot(robot, *leftArm)
	if err != nil {
		return fmt.Errorf("left arm %q: %w", *leftArm, err)
	}
	rightArmRes, err := arm.FromRobot(robot, *rightArm)
	if err != nil {
		return fmt.Errorf("right arm %q: %w", *rightArm, err)
	}

	var leftGripperRes, rightGripperRes gripper.Gripper
	if *leftGripper != "" {
		leftGripperRes, err = gripper.FromRobot(robot, *leftGripper)
		if err != nil {
			logger.Warnf("left gripper %q: %v", *leftGripper, err)
		}
	}
	if *rightGripper != "" {
		rightGripperRes, err = gripper.FromRobot(robot, *rightGripper)
		if err != nil {
			logger.Warnf("right gripper %q: %v", *rightGripper, err)
		}
	}

	motionSvc, err := motion.FromRobot(robot, "builtin")
	if err != nil {
		logger.Warnf("motion service: %v", err)
	}

	// Build dependencies map for the teleop service.
	deps := resource.Dependencies{
		input.Named("left-vive"):  leftCtrl,
		input.Named("right-vive"): rightCtrl,
		arm.Named(*leftArm):       leftArmRes,
		arm.Named(*rightArm):      rightArmRes,
		motion.Named("builtin"):   motionSvc,
	}
	if leftGripperRes != nil {
		deps[gripper.Named(*leftGripper)] = leftGripperRes
	}
	if rightGripperRes != nil {
		deps[gripper.Named(*rightGripper)] = rightGripperRes
	}

	rot := *rotEnabled

	teleopSvc, err := vive.NewTeleopService(ctx, deps, generic.Named("teleop"), &vive.TeleopConfig{
		Hz: *hz,
		Hands: []vive.HandConfig{
			{
				Name:            "left",
				Controller:      "left-vive",
				Arm:             *leftArm,
				Gripper:         *leftGripper,
				Scale:           *scale,
				RotationEnabled: &rot,
				PosDeadzoneMM:   *posDeadzone,
				RotDeadzoneDeg:  *rotDeadzone,
				SmoothAlpha:     *smoothAlpha,
			},
			{
				Name:            "right",
				Controller:      "right-vive",
				Arm:             *rightArm,
				Gripper:         *rightGripper,
				Scale:           *scale,
				RotationEnabled: &rot,
				PosDeadzoneMM:   *posDeadzone,
				RotDeadzoneDeg:  *rotDeadzone,
				SmoothAlpha:     *smoothAlpha,
			},
		},
		CalibrationDir: calibDir,
	}, logger)
	if err != nil {
		return fmt.Errorf("teleop service: %w", err)
	}
	defer teleopSvc.Close(ctx)

	logger.Info("Teleop service started")

	sig := make(chan os.Signal, 1)
	signal.Notify(sig, syscall.SIGINT, syscall.SIGTERM)
	<-sig
	fmt.Println("\n[teleop] Stopped")
	cancel()

	// Force exit if libsurvive cleanup hangs.
	go func() {
		time.Sleep(2 * time.Second)
		fmt.Fprintln(os.Stderr, "[teleop] Forced exit (libsurvive cleanup hung)")
		os.Exit(1)
	}()

	return nil
}
