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

    private PinpointDrive drive;
    private Arm arm;
    private Lift lift;
    private Wrist wrist;
    private Claw claw;

    static final double ARM_TICKS_PER_DEGREE = 28 * 250047.0 / 4913.0 * 100.0 / 20.0 * 1/360.0;
    static final double LIFT_TICKS_PER_MM = (111132.0 / 289.0) / 120.0;

    @Override
    public void runOpMode() {
        Pose2d initialPose = new Pose2d(14.57, 62.61, Math.toRadians(-90));
        drive = new PinpointDrive(hardwareMap, initialPose);
        claw = new Claw(hardwareMap);
        arm = new Arm(hardwareMap);
        wrist = new Wrist(hardwareMap);
        lift = new Lift(hardwareMap);

        TrajectoryActionBuilder goToBasketZero = drive.actionBuilder(initialPose)
                .splineTo(new Vector2d(53, 53), Math.toRadians(45.00));

        TrajectoryActionBuilder goToSample1 = drive.actionBuilder(new Pose2d(53, 53, Math.toRadians(45.00)))
                .turn(Math.toRadians(-130));
        TrajectoryActionBuilder goToBasketOne = drive.actionBuilder(new Pose2d(53, 53, Math.toRadians(-160)))
                .turn(Math.toRadians(170));

        TrajectoryActionBuilder goToSampleTwo = drive.actionBuilder(new Pose2d(53, 53, Math.toRadians(45.00)))
                .turn(Math.toRadians(-110));
        TrajectoryActionBuilder goToBasketTwo = drive.actionBuilder(new Pose2d(53, 53, Math.toRadians(-135)))
                .turn(Math.toRadians(15));

        TrajectoryActionBuilder goToSampleThree = drive.actionBuilder(new Pose2d(53, 53, Math.toRadians(45.00)))
                .turn(Math.toRadians(-95));
        TrajectoryActionBuilder goToBasketThree = drive.actionBuilder(new Pose2d(53, 53, Math.toRadians(-105)))
                .turn(Math.toRadians(132));

        TrajectoryActionBuilder goToAscent = drive.actionBuilder(new Pose2d(53, 60, Math.toRadians(45)))
                .turn(Math.toRadians(160))
                .strafeTo(new Vector2d(20,0));                 ;

        waitForStart();
        if (isStopRequested()) return;

        // Execute each step in the sequence individually using runBlocking
        Actions.runBlocking(claw.closeClaw());
        class drive0 extends Thread{
            public void run() {
                Actions.runBlocking(goToBasketZero.build());
            }
        }
        // Step 2: Follow trajectory
        class UpBPart1 extends Thread {
            public void run() {
                Actions.runBlocking(wrist.setWristPositionAction(0.66));
                Actions.runBlocking(arm.moveArmAction(105, 1));     // Step 3: Lift arm
                Actions.runBlocking(lift.moveSlideAction(640, 1));  // Step 4: Extend slide

            }
        }

        drive0 tobasket = new drive0();
        UpBPart1 score = new UpBPart1();
        tobasket.start();
        score.start();
        try {
            tobasket.join();
            score.join();
        } catch (InterruptedException e){
            //empty
        }
//        Actions.runBlocking(arm.moveArmAction(92, 1));
        upperBasket();

        class turnsample1 extends Thread{
            public void run(){
                Actions.runBlocking(goToSample1.build());
            }
        }
        class armdown extends Thread{
            public void run(){
                Actions.runBlocking(arm.moveArmAction(15, .5));
            }
        }
        turnsample1 turnto = new turnsample1();
        armdown armmove = new armdown();
        turnto.start();
        armmove.start();
        try {
            turnto.join();
            armmove.join();
        } catch (InterruptedException e){
            //empty
        }
        Actions.runBlocking(lift.moveSlideAction(660,0.5));
        Actions.runBlocking(claw.closeClaw());
        sleep(200);
        Actions.runBlocking(arm.moveArmAction(105, 1));
        // Actions.runBlocking(lift.moveSlideAction(0, 0.7));


        // Go To basket
        class drive1 extends Thread{
            public void run(){
                Actions.runBlocking(goToBasketOne.build());
            }
        }
        drive1 tobasket1 = new drive1();
        tobasket1.start();
        score.start();
        try {
            tobasket1.join();
            score.join();
        } catch (InterruptedException e){
            //empty
        }
//        Actions.runBlocking(arm.moveArmAction(92, 1));
        upperBasket();

        Actions.runBlocking(lift.moveSlideAction(485,1));
        class turnsample2 extends Thread{
            public void run() {
                Actions.runBlocking(goToSampleTwo.build());
            }
        }
        //grab
        turnsample2 turnto2 = new turnsample2();
        turnto2.start();
        armmove.start();
        try {
            turnto2.join();
            armmove.join();
        } catch (InterruptedException e){
            //empty
        }

        Actions.runBlocking(lift.moveSlideAction(660,1));
        Actions.runBlocking(claw.closeClaw());
        sleep(200);
        Actions.runBlocking(arm.moveArmAction(105, 1));

        //go to basket
        class drive2 extends Thread{
            public void run(){
                Actions.runBlocking(goToBasketTwo.build());
            }
        }
        drive2 tobasket2 = new drive2();
        tobasket2.start();
        score.start();
        try {
            tobasket2.join();
            score.join();
        } catch (InterruptedException e){
            //empty
        }
//        Actions.runBlocking(arm.moveArmAction(92, 1));
        upperBasket();



        Actions.runBlocking(wrist.setWristPositionAction(0.66));
        class turnsample3 extends Thread{
            public void run(){
                Actions.runBlocking(goToSampleThree.build());
            }
        }
        turnsample3 turnto3 = new turnsample3();
        turnto3.start();
        armmove.start();
        try {
            turnto3.join();
            armmove.join();
        } catch (InterruptedException e){
            //empty
        }


        Actions.runBlocking(arm.moveArmAction(20,1));
        Actions.runBlocking(lift.moveSlideAction(660,1));
        Actions.runBlocking(claw.closeClaw());
        sleep(200);
        Actions.runBlocking(arm.moveArmAction(105, 1));


//        sleep(100);
        class drive3 extends Thread{
            public void run(){
                Actions.runBlocking(goToBasketThree.build());
            }
        }

        drive3 tobasket3 = new drive3();
        tobasket3.start();
        score.start();
        try {
            tobasket2.join();
            score.join();
        } catch (InterruptedException e){
            //empty
        }
        sleep(500);
        upperBasket();


        class retract extends Thread{
            public void run(){
                Actions.runBlocking(lift.moveSlideAction(0, 1));
            }
        }
        class Park extends Thread{
            public void run(){
                Actions.runBlocking(goToAscent.build());
            }
        }
        class Touch extends Thread{
            public void run(){
                Actions.runBlocking(arm.moveArmAction(80,1));
            }
        }
        retract slideClose = new retract();
        Park drivePark = new Park();
        Touch touchbar = new Touch();
        drivePark.start();
        touchbar.start();
        slideClose.start();
        try{
            drivePark.join();
            touchbar.join();
            slideClose.join();
        } catch(InterruptedException e){
            //empty
        }
        // Step 8: Retract slide
    }

    private void upperBasket()
    {
              // Step 5: Position arm

        Actions.runBlocking(arm.moveArmAction(91, 1));
        Actions.runBlocking(claw.openClaw());
        sleep(200);
        Actions.runBlocking(arm.moveArmAction(105, 1));
        Actions.runBlocking(lift.moveSlideAction(475, 1));    // Step 8: Retract slide
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
            return new ClawPositionAction(0.526);
        }

        public Action openClaw() {
            return new ClawPositionAction(0.75); //0.6294
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