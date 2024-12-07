package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="Degrees", group="Calibration")
public class Calibration extends LinearOpMode {

    // Declare OpMode members
    private DcMotorEx slideMotor = null;
    private DcMotor armMotor = null;
    private Servo wrist = null;
    private Servo claw = null;  // Replacing intake wheel with claw mechanism

    // Constants for arm and lift movements
    final double ARM_TICKS_PER_DEGREE = 28 * 250047.0 / 4913.0 * 100.0 / 20.0 * 1/360.0;
    final double LIFT_TICKS_PER_MM = (111132.0 / 289.0) / 120.0;

    // Variables for tracking positions
    private double armPositionDegrees = 0; // Using degrees for arm position
    private double slidePositionMM = 0;    // Using millimeters for slide position
    private double wristPosition = 0.98;   // Wrist servo position
    private double clawPosition = 0.526;     // Initial position for the claw

    @Override
    public void runOpMode() {
        // Initialize hardware
        slideMotor = hardwareMap.get(DcMotorEx.class, "slideMotor");
        armMotor = hardwareMap.get(DcMotor.class, "armMotor");
        wrist = hardwareMap.get(Servo.class, "wrist");
        claw = hardwareMap.get(Servo.class, "claw"); // Initialize the claw servo

        // Set initial positions for arm and slide
        slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slideMotor.setDirection(DcMotor.Direction.REVERSE);

        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Wait for the game driver to press play
        waitForStart();

        // Main OpMode loop
        while (opModeIsActive()) {
            // Slide control using bumpers
            if (gamepad1.right_bumper) {
                slidePositionMM += 5; // Raise slide
            } else if (gamepad1.left_bumper) {
                slidePositionMM -= 5; // Lower slide
            }
            slidePositionMM = Range.clip(slidePositionMM, 0, 600); // Limit slide position
            int slideTargetPosition = (int) (slidePositionMM * LIFT_TICKS_PER_MM); // Convert mm to ticks
            slideMotor.setTargetPosition(slideTargetPosition);
            slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            slideMotor.setPower(0.5);

            // Arm control using D-pad up/down
            if (gamepad1.dpad_up) {
                armPositionDegrees += 2; // Raise arm
            } else if (gamepad1.dpad_down) {
                armPositionDegrees -= 2; // Lower arm
            }
            armPositionDegrees = Range.clip(armPositionDegrees, 0, 106); // Limit arm position
            int armTargetPosition = (int) (armPositionDegrees * ARM_TICKS_PER_DEGREE); // Convert degrees to ticks
            armMotor.setTargetPosition(armTargetPosition);
            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            armMotor.setPower(0.5);

            // Wrist control using D-pad left/right with torque-based servo handling
            if (gamepad1.dpad_left) {
                wristPosition = Math.max(0, wristPosition - 0.01); // Move wrist in
            } else if (gamepad1.dpad_right) {
                wristPosition = Math.min(1, wristPosition + 0.01); // Move wrist out
            }
            wrist.setPosition(wristPosition);

            // Claw control using left stick Y axis
            double clawControl = gamepad1.left_stick_y;
            clawPosition += clawControl * 0.01; // Adjust sensitivity as needed
            clawPosition = Range.clip(clawPosition, 0, 1); // Limit claw position
            claw.setPosition(clawPosition); // Set claw position

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