package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name="IntoTheDeepAuto", group="Autonomous")
public class IntoTheDeepAuto extends LinearOpMode {

    private MecanumDrive drive;
    private Arm arm;
    private Lift lift;
    private Wrist wrist;
    private Claw claw;

    static final double ARM_TICKS_PER_DEGREE = 28 * 250047.0 / 4913.0 * 100.0 / 20.0 * 1/360.0;
    static final double LIFT_TICKS_PER_MM = (111132.0 / 289.0) / 120.0;

    @Override
    public void runOpMode() {
        Pose2d initialPose = new Pose2d(14.57, 62.61, Math.toRadians(-89.32));
        drive = new MecanumDrive(hardwareMap, initialPose);
        claw = new Claw(hardwareMap);
        arm = new Arm(hardwareMap);
        wrist = new Wrist(hardwareMap);
        lift = new Lift(hardwareMap);

        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose)
                .splineTo(new Vector2d(53.21, 53.21), Math.toRadians(45.00))  // Point exactly toward the red team wall corner
                .waitSeconds(3);

       /* TrajectoryActionBuilder tab2 = drive.actionBuilder(initialPose)
                .lineToY(37)
                .setTangent(Math.toRadians(0))
                .lineToX(18)
                .waitSeconds(3)
                .setTangent(Math.toRadians(0))
                .lineToXSplineHeading(46, Math.toRadians(180))
                .waitSeconds(3);

        TrajectoryActionBuilder tab3 = drive.actionBuilder(initialPose)
                .lineToYSplineHeading(33, Math.toRadians(180))
                .waitSeconds(2)
                .strafeTo(new Vector2d(46, 30))
                .waitSeconds(3);

        TrajectoryActionBuilder tab4 = drive.actionBuilder(initialPose)
                .lineToYSplineHeading(33, Math.toRadians(0))
                .waitSeconds(2)
                .setTangent(Math.toRadians(90))
                .lineToY(48)
                .setTangent(Math.toRadians(0))
                .lineToX(32)
                .strafeTo(new Vector2d(44.5, 30))
                .turn(Math.toRadians(180))
                .lineToX(47.5)
                .waitSeconds(3);

        TrajectoryActionBuilder trajacotrySample = drive.actionBuilder(new Pose2d(-60, -55, Math.toRadians(225))) // Start from the last pose
                .turn(Math.toRadians(-135))  // Rotate by -135 degrees to face the wall between red and blue alliances
                .waitSeconds(3);


        Action trajectoryActionCloseOut = tab1.fresh()
                .strafeTo(new Vector2d(48, 12))
                .build();
        */
        int visionOutputPosition = getVisionOutput();
        telemetry.addData("Starting Position", visionOutputPosition);
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        // Select trajectory based on vision position
        Action selectedTrajectoryAction;
        selectedTrajectoryAction = tab1.build();
        /* if (visionOutputPosition == 1) {
            selectedTrajectoryAction = tab1.build();
        }  else if (visionOutputPosition == 2) {
            selectedTrajectoryAction = tab2.build();
        } else {
            selectedTrajectoryAction = tab3.build();
        } */

        // Execute the selected sequence of actions
        Actions.runBlocking(
                new SequentialAction(
                        claw.closeClaw(), // Step 1: Close the claw
                        new ParallelAction(
                                selectedTrajectoryAction, // Step 2: Move along the trajectory while performing actions in parallel
                                wrist.setWristPositionAction(0.3), // Set wrist position
                                arm.moveArmAction(100, 0.5),       // Lift the arm
                                lift.moveSlideAction(600, 0.5)     // Extend the slide
                        ),
                        claw.openClaw(), // Step 3: Open the claw upon reaching the destination
                        claw.closeClaw(), // Step 4: Close the claw again
                        new ParallelAction(
                                lift.moveSlideAction(0, 0.5),     // Retract the slide to position 0
                                arm.moveArmAction(0, 0.5)  // Bring down the arm to position 0
           //                     trajacotrySample.build()
                        )
                )
        );

    }

    // Classes for Arm, Lift, Wrist, and Claw with action-based methods
    public class Arm {
        private DcMotorEx armMotor;

        public Arm(HardwareMap hardwareMap) {
            armMotor = hardwareMap.get(DcMotorEx.class, "armMotor");
            armMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        }

        public Action moveArmAction(double degrees, double power) {
            int targetPosition = (int) (degrees * ARM_TICKS_PER_DEGREE);
            armMotor.setTargetPosition(targetPosition);
            armMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

            return new Action() {
                @Override
                public boolean run(@NonNull TelemetryPacket packet) {
                    armMotor.setPower(power);
                    packet.put("Arm Position", armMotor.getCurrentPosition());

                    if (!armMotor.isBusy()) {
                        armMotor.setPower(0);
                        return false;  // Action is complete
                    }
                    return true;  // Action is still running
                }
            };
        }
    }

    public class Lift {
        private DcMotorEx slideMotor;

        public Lift(HardwareMap hardwareMap) {
            slideMotor = hardwareMap.get(DcMotorEx.class, "slideMotor");
            slideMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            slideMotor.setDirection(DcMotorEx.Direction.REVERSE);
            slideMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            slideMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        }

        public Action moveSlideAction(double mm, double power) {
            int targetPosition = (int) (mm * LIFT_TICKS_PER_MM);
            slideMotor.setTargetPosition(targetPosition);
            slideMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

            return new Action() {
                @Override
                public boolean run(@NonNull TelemetryPacket packet) {
                    slideMotor.setPower(power);
                    packet.put("Slide Position", slideMotor.getCurrentPosition());

                    if (!slideMotor.isBusy()) {
                        slideMotor.setPower(0);
                        return false;  // Action is complete
                    }
                    return true;  // Action is still running
                }
            };
        }
    }

    public class Wrist {
        private Servo wrist;

        public Wrist(HardwareMap hardwareMap) {
            wrist = hardwareMap.get(Servo.class, "wrist");
        }

        public Action setWristPositionAction(double position) {
            return new Action() {
                @Override
                public boolean run(@NonNull TelemetryPacket packet) {
                    wrist.setPosition(position);
                    packet.put("Wrist Position", wrist.getPosition());
                    return false;  // Immediate completion for simplicity
                }
            };
        }
    }

    public class Claw {
        private Servo claw;

        public Claw(HardwareMap hardwareMap) {
            claw = hardwareMap.get(Servo.class, "claw");
        }

        public Action closeClaw() {
            return new Action() {
                @Override
                public boolean run(@NonNull TelemetryPacket packet) {
                    claw.setPosition(0.7030);
                    return false;  // Immediate completion for simplicity
                }
            };
        }

        public Action openClaw() {
            return new Action() {
                @Override
                public boolean run(@NonNull TelemetryPacket packet) {
                    claw.setPosition(1.0);
                    return false;  // Immediate completion
                }
            };
        }
    }

    private int getVisionOutput() {
        // Placeholder for vision processing, returning a mock position.
        return 1;
    }
}