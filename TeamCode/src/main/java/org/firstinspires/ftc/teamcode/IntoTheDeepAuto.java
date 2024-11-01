package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
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
        Pose2d initialPose = new Pose2d(14.57, 62.61, Math.toRadians(-90));
        drive = new MecanumDrive(hardwareMap, initialPose);
        claw = new Claw(hardwareMap);
        arm = new Arm(hardwareMap);
        wrist = new Wrist(hardwareMap);
        lift = new Lift(hardwareMap);

        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose)
                .splineTo(new Vector2d(53.21, 53.21), Math.toRadians(45.00));

        TrajectoryActionBuilder tab2 = drive.actionBuilder(new Pose2d(53.21, 53.21, Math.toRadians(45.00)))
                .splineTo(new Vector2d(47.77, 40.62), Math.toRadians(270.00));

        TrajectoryActionBuilder tab3 = drive.actionBuilder(new Pose2d(47.77, 40.62, Math.toRadians(270.00)))
                .splineTo(new Vector2d(53.45, 53.45), Math.toRadians(45.00));

        TrajectoryActionBuilder tab4 = drive.actionBuilder(new Pose2d(53.21, 53.21, Math.toRadians(45.00)))
                .setReversed(true)
                .splineTo(new Vector2d(38.12, 34.68), Math.toRadians(240.73))
                .setReversed(false);
               // .lineTo(new Vector2d(31.02, 12.71));

        waitForStart();
        if (isStopRequested()) return;

        // Execute each step in the sequence individually using runBlocking
        Actions.runBlocking(claw.closeClaw());                // Step 1: Close the claw
        Actions.runBlocking(tab1.build());                    // Step 2: Follow trajectory
        upperBasket();
        Actions.runBlocking(tab2.build());
        Actions.runBlocking(arm.moveArmAction(5, 0.5));
        Actions.runBlocking(claw.closeClaw());
        sleep(250);
        Actions.runBlocking(arm.moveArmAction(80, 0.5));
        Actions.runBlocking(lift.moveSlideAction(0, 0.7));
        // Go To basket
       Actions.runBlocking(tab3.build());
       upperBasketSecond();
                  // Step 8: Retract slide
    }

    private void upperBasket()
    {
        Actions.runBlocking(arm.moveArmAction(100, 0.6));     // Step 3: Lift arm
        Actions.runBlocking(wrist.setWristPositionAction(0.37));
        Actions.runBlocking(lift.moveSlideAction(655, 0.7));  // Step 4: Extend slide
        Actions.runBlocking(arm.moveArmAction(88, 0.6));      // Step 5: Position arm
        Actions.runBlocking(claw.openClaw());
        sleep(100);
        Actions.runBlocking(arm.moveArmAction(98, 0.5));
        Actions.runBlocking(lift.moveSlideAction(270, 0.5));    // Step 8: Retract slide
    }

    private void upperBasketSecond()
    {
        Actions.runBlocking(wrist.setWristPositionAction(0));
        Actions.runBlocking(arm.moveArmAction(105, 0.6));     // Step 3: Lift arm
        Actions.runBlocking(wrist.setWristPositionAction(0.37));
        Actions.runBlocking(lift.moveSlideAction(655, 0.7));  // Step 4: Extend slide
        Actions.runBlocking(arm.moveArmAction(90, 0.5));      // Step 5: Position arm
        Actions.runBlocking(claw.openClaw());
        sleep(100);
        Actions.runBlocking(arm.moveArmAction(98, 0.5));
        Actions.runBlocking(lift.moveSlideAction(0, 0.6));    // Step 8: Retract slide
        Actions.runBlocking(arm.moveArmAction(0, 0.6));
    }

    public class Arm {
        private DcMotorEx armMotor;

        public Arm(HardwareMap hardwareMap) {
            armMotor = hardwareMap.get(DcMotorEx.class, "armMotor");
            armMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        public Action moveArmAction(double degrees, double power) {
            int targetPosition = (int) (degrees * ARM_TICKS_PER_DEGREE);
            armMotor.setTargetPosition(targetPosition);
            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            armMotor.setPower(power);

            return new ArmMoveAction(power, targetPosition);
        }

        private class ArmMoveAction implements Action {
            private final double power;
            private final int targetPosition;
            private boolean initialized = false;

            public ArmMoveAction(double power, int targetPosition) {
                this.power = power;
                this.targetPosition = targetPosition;
            }

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    armMotor.setPower(power);
                    initialized = true;
                }

                packet.put("Arm Position", armMotor.getCurrentPosition());
                packet.put("Arm Target Position", targetPosition);
                packet.put("Arm Power", armMotor.getPower());

                if (Math.abs(targetPosition - armMotor.getCurrentPosition()) < 10) {
                    armMotor.setPower(0);
                    return false;
                }
                return true;
            }
        }
    }

    public class Lift {
        private DcMotorEx slideMotor;

        public Lift(HardwareMap hardwareMap) {
            slideMotor = hardwareMap.get(DcMotorEx.class, "slideMotor");
            slideMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            slideMotor.setDirection(DcMotorEx.Direction.REVERSE);
            slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            slideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        public Action moveSlideAction(double mm, double power) {
            int targetPosition = (int) (mm * LIFT_TICKS_PER_MM);
            slideMotor.setTargetPosition(targetPosition);
            slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            slideMotor.setPower(power);

            return new SlideMoveAction(power, targetPosition);
        }

        private class SlideMoveAction implements Action {
            private final double power;
            private final int targetPosition;
            private boolean initialized = false;

            public SlideMoveAction(double power, int targetPosition) {
                this.power = power;
                this.targetPosition = targetPosition;
            }

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    slideMotor.setPower(power);
                    initialized = true;
                }

                packet.put("Slide Position", slideMotor.getCurrentPosition());
                packet.put("Slide Target Position", targetPosition);

                if (Math.abs(targetPosition - slideMotor.getCurrentPosition()) < 10) {
                    slideMotor.setPower(0);
                    return false;
                }
                return true;
            }
        }
    }

    public class Wrist {
        private Servo wrist;

        public Wrist(HardwareMap hardwareMap) {
            wrist = hardwareMap.get(Servo.class, "wrist");
        }

        public Action setWristPositionAction(double position) {
            return new WristPositionAction(position);
        }

        private class WristPositionAction implements Action {
            private final double position;

            public WristPositionAction(double position) {
                this.position = position;
            }

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                wrist.setPosition(position);
                packet.put("Wrist Position", wrist.getPosition());
                return false;
            }
        }
    }

    public class Claw {
        private Servo claw;

        public Claw(HardwareMap hardwareMap) {
            claw = hardwareMap.get(Servo.class, "claw");
        }

        public Action closeClaw() {
            return new ClawPositionAction(0.46);
        }

        public Action openClaw() {
            return new ClawPositionAction(0.8);
        }

        private class ClawPositionAction implements Action {
            private final double position;

            public ClawPositionAction(double position) {
                this.position = position;
            }

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                claw.setPosition(position);
                return false;
            }
        }
    }
}