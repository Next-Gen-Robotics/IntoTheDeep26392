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

@Autonomous(name="Specimen", group="Autonomous")
public class Specimen extends LinearOpMode {

    private PinpointDrive drive;
    private Arm arm;
    private Lift lift;
    private Wrist wrist;
    private Claw claw;

    static final double ARM_TICKS_PER_DEGREE = 28 * 250047.0 / 4913.0 * 100.0 / 20.0 * 1/360.0;
    static final double LIFT_TICKS_PER_MM = (111132.0 / 289.0) / 120.0;

    @Override
    public void runOpMode() {
        Pose2d initialPose = new Pose2d(-9.50, 63.53, Math.toRadians(270));
        drive = new PinpointDrive(hardwareMap, initialPose);
        claw = new Claw(hardwareMap);
        arm = new Arm(hardwareMap);
        wrist = new Wrist(hardwareMap);
        lift = new Lift(hardwareMap);

        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose)
                .lineToY(40);

        // Go to first sample
        TrajectoryActionBuilder tab2 = drive.actionBuilder(new Pose2d(-9.50, 40, Math.toRadians(270)))
                .strafeTo(new Vector2d(-47,40));
        // turn to drop first sample
        TrajectoryActionBuilder tab3 = drive.actionBuilder(new Pose2d(-47, 40, Math.toRadians(270)))
                .turn(Math.toRadians(160));
        //turn and go to hang second sample
        TrajectoryActionBuilder tosubmersible1 = drive.actionBuilder(new Pose2d(-47,40, Math.toRadians(90)))
                .turn(Math.toRadians(-165))
                .strafeTo(new Vector2d(-5,40));
        TrajectoryActionBuilder tab4 = drive.actionBuilder(new Pose2d(-5, 40, Math.toRadians(270)))
                .lineToY(54)
                .strafeTo(new Vector2d(-56,54));
        // turn to drop second sample
        TrajectoryActionBuilder tab5 = drive.actionBuilder(new Pose2d(-58.5, 40, Math.toRadians(270)))
                .turn(Math.toRadians(160));
        // turn for parking
        TrajectoryActionBuilder tab6 = drive.actionBuilder(new Pose2d(-67.77, 40, Math.toRadians(110)))
                .turn(Math.toRadians(-160))
                .strafeTo(new Vector2d(-79.77,55.62));

        waitForStart();
        if (isStopRequested()) return;

        //hold preloaded specimen before doing anything
        Actions.runBlocking(claw.closeClaw());

        class DriveZero extends Thread {
            public void run() {
                Actions.runBlocking(tab1.build());
            }
        }

        class HangSpecimenZero extends Thread {
            public void run() {
                Actions.runBlocking(arm.moveArmAction(68, 1));// Step 2: Follow trajectory
                Actions.runBlocking(wrist.setWristPositionAction(0.66));
                Actions.runBlocking(lift.moveSlideAction(130, 1));
            }
        }

        DriveZero driveZero= new DriveZero();
        HangSpecimenZero hangSpecimenZero= new HangSpecimenZero();
        hangSpecimenZero.start();
        driveZero.start();;
        try {
            hangSpecimenZero.join();
            driveZero.join();;
        } catch (InterruptedException e) {
            //empty
        }

        hangSample();
        //end of hanging preloaded specimen
        // go and pickup sample from the floor
        Actions.runBlocking(tab2.build());
        pickupSample();
        Actions.runBlocking(arm.moveArmAction(39,1));
        // Turn left and go to observation zone and drop sample
        Actions.runBlocking(tab3.build());
        pickupSpecimen();
        //Actions.runBlocking(arm.moveArmAction(66, 1));// Step 2: Follow trajectory
        Actions.runBlocking(lift.moveSlideAction(130, 1));
        Actions.runBlocking(tosubmersible1.build());
        //sleep(1000);
        Actions.runBlocking(lift.moveSlideAction(120, 1));
        hangSample();
        Actions.runBlocking(tab4.build());
        Actions.runBlocking(arm.moveArmAction(20, 1));
        Actions.runBlocking(lift.moveSlideAction(610, 1));
        Actions.runBlocking(claw.closeClaw());
        sleep(250);
        Actions.runBlocking(lift.moveSlideAction(0, 1));
        Actions.runBlocking(arm.moveArmAction(0, 1));
        //sleep(1000);
        /* hangSample();
        Actions.runBlocking(tab4.build());
        Actions.runBlocking(arm.moveArmAction(35,0.5));
        pickupSecondSample();
        Actions.runBlocking(tab5.build());
        pickupSpecimen();
        Actions.runBlocking(lift.moveSlideAction(0,0.5));
        Actions.runBlocking(arm.moveArmAction(0,0.5));
        Actions.runBlocking(tab6.build());
        */
    }
    // it drops sample and pickups specimen
    private void pickupSpecimen()
    {
        //drop sample and pickup specimen and go to submersbile
        Actions.runBlocking(lift.moveSlideAction(235, 1));
        Actions.runBlocking(claw.openClaw());
        // sleep(100);
        Actions.runBlocking(arm.moveArmAction(39,1));
        Actions.runBlocking(lift.moveSlideAction(465,1));
        //Actions.runBlocking(wrist.setWristPositionAction(0.66));
        Actions.runBlocking(claw.closeClaw());
        sleep(200);
        Actions.runBlocking(arm.moveArmAction(100,1));
//        Actions.runBlocking(lift.moveSlideAction(70, 1));
        //sleep(250);

    }
    private  void hangSample()
    {
        Actions.runBlocking(arm.moveArmAction(58, 1));// Step 2: Follow trajectory
        Actions.runBlocking(lift.moveSlideAction(290, 1));
        Actions.runBlocking(claw.openClaw());
        sleep(250);
        Actions.runBlocking(lift.moveSlideAction(0, 1));
    }

    private void pickupSample()
    {
        //pickup sample
        Actions.runBlocking(lift.moveSlideAction(0, 1));
        Actions.runBlocking(arm.moveArmAction(0, 1));
        Actions.runBlocking(claw.closeClaw());
        sleep(300);
        Actions.runBlocking(arm.moveArmAction(20, 1));
        //Actions.runBlocking(lift.moveSlideAction(300, 0.5));
    }

    private void pickupSecondSample()
    {
        //pickup sample
        Actions.runBlocking(lift.moveSlideAction(280, 0.5));
        Actions.runBlocking(arm.moveArmAction(5, 0.5));
        Actions.runBlocking(claw.closeClaw());
        sleep(500);
        Actions.runBlocking(arm.moveArmAction(20, 0.5));
        // Actions.runBlocking(lift.moveSlideAction(300, 0.5));
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