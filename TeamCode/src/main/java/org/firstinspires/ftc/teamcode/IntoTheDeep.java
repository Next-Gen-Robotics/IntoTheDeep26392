package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="IntoTheDeep", group="TeleOp")
public class IntoTheDeep extends LinearOpMode {

    // Declare OpMode members
    private MecanumDrive drive;
    private DcMotorEx slideMotor = null;
    private DcMotor armMotor = null;
    private Servo wrist = null;
    private Servo claw = null;

    // Constants for arm and lift movements
    final double ARM_TICKS_PER_DEGREE = 28 * 250047.0 / 4913.0 * 100.0 / 20.0 * 1/360.0;
    final double LIFT_TICKS_PER_MM = (111132.0 / 289.0) / 120.0;

    // Variables for tracking positions
    private double armPositionDegrees = 85; // Using degrees for arm position
    private double slidePositionMM = 0.75;    // Using millimeters for slide position
    private double wristPosition = 0.66;      // Wrist servo position
    private final double joystickThreshold = 0.05; // Threshold to ignore minor joystick inputs

    @Override
    public void runOpMode() {
        // Initialize hardware and PinpointDrive
        drive = new PinpointDrive(hardwareMap, new Pose2d(-24, -72, Math.toRadians(180)));
        slideMotor = hardwareMap.get(DcMotorEx.class, "slideMotor");
        armMotor = hardwareMap.get(DcMotor.class, "armMotor");
        wrist = hardwareMap.get(Servo.class, "wrist");
        claw = hardwareMap.get(Servo.class, "claw"); // Initialize the claw servo

        // Set the motors to brake when zero power is applied
        slideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Set initial positions for arm and slide
//        slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slideMotor.setDirection(DcMotor.Direction.REVERSE);

        // armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Wait for the game driver to press play
        waitForStart();

        // Main OpMode loop
        while (opModeIsActive()) {
            // Drivetrain controls using PinpointDrive
            double y = -gamepad1.left_stick_y; // Forward/backward
            double x = - gamepad1.left_stick_x;  // Left/right strafing
            double rx = - gamepad1.right_stick_x; // Rotation

            // Only move if joystick input exceeds the threshold
            if (Math.abs(y) > joystickThreshold || Math.abs(x) > joystickThreshold || Math.abs(rx) > joystickThreshold) {
                PoseVelocity2d drivePowers = new PoseVelocity2d(new Vector2d(y, x), rx);
                drive.setDrivePowers(drivePowers);
            } else {
                // Stop the drivetrain if joystick is within the deadzone
                drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));
            }

            // Slide control using bumpers
            if (gamepad1.right_bumper) {
                slidePositionMM += 20; // Raise slide
            } else if (gamepad1.left_bumper) {
                slidePositionMM -= 20; // Lower slide
            }
            slidePositionMM = Range.clip(slidePositionMM, 0, 650); // Limit slide position
            int slideTargetPosition = (int) (slidePositionMM * LIFT_TICKS_PER_MM); // Convert mm to ticks
            slideMotor.setTargetPosition(slideTargetPosition);
            slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            slideMotor.setPower(1);

            // Arm control using D-pad up/down
            if (gamepad1.dpad_up) {
                armPositionDegrees += 2; // Raise arm
            } else if (gamepad1.dpad_down) {
                armPositionDegrees -= 2; // Lower arm
            }
            armPositionDegrees = Range.clip(armPositionDegrees, 0, 115); // Limit arm position
            int armTargetPosition = (int) (armPositionDegrees * ARM_TICKS_PER_DEGREE); // Convert degrees to ticks
            armMotor.setTargetPosition(armTargetPosition);
            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            armMotor.setPower(0.5);

            // Wrist control using D-pad left/right
            if (gamepad1.dpad_right) {
                wristPosition = Math.max(0.32, wristPosition - 0.01); // Move wrist in
            } else if (gamepad1.dpad_left) {
                wristPosition = Math.min(1, wristPosition + 0.01); // Move wrist out
            }
            wrist.setPosition(wristPosition);

            // Claw control using A (close), X (neutral), and B (open)
            if (gamepad1.b) {
                claw.setPosition(0.7);  // Open the claw
            } else if (gamepad1.a) {
                claw.setPosition(0.46);  // Close the claw
            } else if (gamepad1.x) {
                claw.setPosition(0.49);  // Neutral position
            }

            // Telemetry for debugging and calibration
            telemetry.addData("Slide Position (mm):", slidePositionMM);
            telemetry.addData("Slide Target Position (ticks):", slideMotor.getTargetPosition());
            telemetry.addData("Slide Current Position (ticks):", slideMotor.getCurrentPosition());

            telemetry.addData("Arm Position (degrees):", armPositionDegrees);
            telemetry.addData("Arm Target Position (ticks):", armMotor.getTargetPosition());
            telemetry.addData("Arm Current Position (ticks):", armMotor.getCurrentPosition());

            telemetry.addData("Wrist Position", wrist.getPosition());
            telemetry.addData("Claw Position", claw.getPosition());  // Added claw telemetry
            telemetry.update();
        }
    }
}