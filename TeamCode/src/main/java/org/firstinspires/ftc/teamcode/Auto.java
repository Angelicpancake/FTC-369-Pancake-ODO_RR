package org.firstinspires.ftc.teamcode;

// RR-specific imports

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;

import java.util.*;
import java.math.*;


//non rr imports
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


@Config
@Autonomous(name = "MALEK AUTONOMOUS", group = "Autonomous")
public class Auto extends LinearOpMode {

    private int chamberHeight = 2075;
    private int clipOnChamberHeight = 750;
    private int grabFromWallHeight = 50;

    private int armDownPosition = 1820;
    private int armBackPosition = 180;
    private int armReadyToGoDownPosition = 1250;
    private int armUpPosition = 1045;



    //classes for non drivetrain mechanisms
    public class ArmClaw {
        private Servo armClaw;
        ElapsedTime timer = new ElapsedTime();



        public ArmClaw(HardwareMap hardwareMap) {
            armClaw = hardwareMap.get(Servo.class, "armClaw");
            //telemetry.addData(Double.toString(armClaw.getPosition()));


        }

        public class CloseArmClaw implements Action {
            private  boolean isReset = false;
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!isReset) {
                    isReset = true;
                    timer.reset();
                }
                armClaw.setPosition(0.9);


                if(timer.seconds() > 0.5) {
                    return false;
                }

                return true;


            }
        }
        public Action closeArmClaw() {
            return new CloseArmClaw();
        }

        public class OpenArmClaw implements Action {

            private  boolean isReset = false;
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!isReset) {
                    isReset = true;
                    timer.reset();
                }
                armClaw.setPosition(0.1);


                if(timer.seconds() > 0.5) {
                    return false;
                }

                return true;
            }
        }
        public Action openArmClaw() {
            return new OpenArmClaw();
        }
    }
    // slide class
    public class Slide{
        public DcMotor slide;

        public Slide(HardwareMap hardwareMap){
            slide = hardwareMap.get(DcMotor.class, "slide");
            slide.setDirection(DcMotorSimple.Direction.REVERSE);
            slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        public class SlideUp implements Action{

            private boolean initialized = false;


            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                slide.setTargetPosition(chamberHeight);
                slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                slide.setPower(1);

                return Math.abs(slide.getTargetPosition()-slide.getCurrentPosition())>5;

            }


        }


        // implements medium position for the slide
        public class SlideMidDown implements Action{

            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                slide.setTargetPosition(clipOnChamberHeight);
                slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                slide.setPower(0.7);

                return Math.abs(slide.getTargetPosition()-slide.getCurrentPosition())>5;
            }

        }

        public Action slideUp(){
            return new SlideUp();
        }
        public Action slideMidDown(){
            return new SlideMidDown();
        }

        public class SlideDown implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                slide.setTargetPosition(grabFromWallHeight);
                slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                slide.setPower(1);

                return Math.abs(slide.getTargetPosition()-slide.getCurrentPosition())>5;

            }
        }
        public Action slideDown(){
            return new SlideDown();
        }
    }


    public class SlideClaw {
        private Servo slideClaw;

        public SlideClaw(HardwareMap hardwareMap) {
            slideClaw = hardwareMap.get(Servo.class, "slideClaw");
        }

        public class CloseSlideClaw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                slideClaw.setPosition(0.75);
                return false;
            }
        }
        public Action closeSlideClaw() {
            return new CloseSlideClaw();
        }

        public class OpenSlideClaw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                slideClaw.setPosition(0.1);
                return false;
            }
        }
        public Action openSlideClaw() {
            return new OpenSlideClaw();
        }
    }


    /////////ARM CODE BELOW
    public class Arm{
        public DcMotor arm;

        public Arm(HardwareMap hardwareMap){
            arm = hardwareMap.get(DcMotor.class, "arm");
            // uncomment to reverse arm if needed
            arm.setDirection(DcMotorSimple.Direction.REVERSE);
            arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        public class ArmDown implements Action{

            private boolean initialized = false;


            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                arm.setTargetPosition(armDownPosition);
                arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                arm.setPower(0.6);

                return Math.abs(arm.getTargetPosition() - arm.getCurrentPosition())>5;

            }


        }


        public Action armDown(){
            return new ArmDown();
        }

        public class ArmUp implements Action{
            private boolean initialized = false;


            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                arm.setTargetPosition(armUpPosition);
                arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                arm.setPower(0.7);

                return Math.abs(arm.getTargetPosition() - arm.getCurrentPosition()) > 5;
            }
        }

        public Action armUp(){
            return new ArmUp();
        }

        public class ArmReady implements Action{
            private boolean initialized = false;


            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                arm.setTargetPosition(armReadyToGoDownPosition);
                arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                arm.setPower(0.7);

                return Math.abs(arm.getTargetPosition() - arm.getCurrentPosition()) > 5;
            }
        }
        public Action armReady(){
            return new ArmReady();
        }

        public class ArmBack implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                arm.setTargetPosition(armBackPosition);
                arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                arm.setPower(0.7);

                return Math.abs(arm.getTargetPosition()  -  arm.getCurrentPosition())>5;

            }
        }
        public Action armBack(){
            return new ArmBack();
        }
    }
    /////////ARM CODE ABOVE








    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d startPose1 = new Pose2d(10, -62, 3*Math.PI/2);
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose1);




        Slide slide = new Slide(hardwareMap);
        SlideClaw slideClaw = new SlideClaw(hardwareMap);

        Arm arm = new Arm(hardwareMap);
        ArmClaw armClaw1 = new ArmClaw(hardwareMap);



        // current path - go to meep meep file to adjust it and test it
        // current math fromm meep meep visualizer, adjust as needed inmeep meep file, then test, than use here
//        TrajectoryActionBuilder part1 = drive.actionBuilder(startPose1)
//                .lineToY(-33.5)  // Drive forward
//                // attach specimen
//                .lineToY(-40)  // Move back a bit
//
//
//
//                //spline to go to the three smaples
//                .setTangent(0)
//                .lineToX(20)
//                .splineToConstantHeading(new Vector2d(45, -9), Math.PI/4)
//
//
//
//                //push first sample down to obervation zone
//                .setTangent(Math.PI/2)
//                .lineToY(-52)
//
//
//                // go to second sample
//                .lineToY(-12)
//                .setTangent(0)
//                .lineToX(56)
//                .setTangent(Math.PI/2)
//                .lineToY(-52)
//
//
//                //diagonal to get out of obervation zone and get ready to recive specimen
//                .setTangent(-0.25)
//                .lineToXLinearHeading(38,Math.PI/2)
//
//
//                //clip trial #1
//                .setTangent(Math.PI/2)
//                .lineToYLinearHeading(-60, Math.PI/2)
//                .strafeToLinearHeading(new Vector2d(7, -33.5), 3*Math.PI/2)
//                .strafeToLinearHeading(new Vector2d(38,-60), Math.PI/2)
//
//                //clip #2
//                .strafeToLinearHeading(new Vector2d(4, -33.5), 3*Math.PI/2)
//                .strafeToLinearHeading(new Vector2d(38,-60), Math.PI/2)
//
//                //clip #3
//                .strafeToLinearHeading(new Vector2d(1, -33.5), 3*Math.PI/2)
//
//                //park - enable if extra time left
//                //.setTangent(2.6)
//                //.lineToXLinearHeading(48, Math.PI/2)
//
//                .build());


        // actions that need to happen on init; for instance, a claw tightening.
        //:TODO add any actions that need to happen on init

        // move to chamber and clip #1
//        TrajectoryActionBuilder TestTurning = drive.actionBuilder(startPose1)
//                .turnTo(new Rotation2d(drive.getPoseEstimate().heading.toDouble(), Math.toRadians(180)));

        TrajectoryActionBuilder forawrdDrive1 = drive.actionBuilder(startPose1)
                .strafeTo(new Vector2d(-2,-26.5));

        // move back so we can strafe to samples
        TrajectoryActionBuilder moveBack1 = drive.actionBuilder(drive.getPoseEstimate())
                .lineToY(-45);

        TrajectoryActionBuilder strafeToFirstSample = drive.actionBuilder(drive.getPoseEstimate())
                .strafeToLinearHeading(new Vector2d(71,-34.5), Math.toRadians(115));

        TrajectoryActionBuilder moveBackToDropFirstSample = drive.actionBuilder(new Pose2d(71,-34.5,Math.toRadians(115)))
                .strafeToConstantHeading(new Vector2d(71,-44));

        TrajectoryActionBuilder moveToSecondSample = drive.actionBuilder(new Pose2d(71,-44,drive.getPoseEstimate().heading.toDouble()))
                .strafeToLinearHeading(new Vector2d(78,-39), Math.toRadians(115));


        TrajectoryActionBuilder moveBackToDropSecondSample = drive.actionBuilder(new Pose2d(78,-39,drive.getPoseEstimate().heading.toDouble()))
                .strafeToConstantHeading(new Vector2d(78, -48));

        TrajectoryActionBuilder moveAndAimAtThirdSample = drive.actionBuilder(drive.getPoseEstimate())
                .strafeToLinearHeading(new Vector2d(85,-43),Math.PI/2.5);

        TrajectoryActionBuilder moveBackToDropThirdSample = drive.actionBuilder(new Pose2d(60,-43,Math.PI/2.5))
                .strafeToConstantHeading(new Vector2d(85,-50));

        TrajectoryActionBuilder goToGrabFirst = drive.actionBuilder(new Pose2d(60,-50,Math.PI/2.5))
                .strafeToLinearHeading(new Vector2d(37, -60), Math.PI/2);

        TrajectoryActionBuilder goToClipFirst = drive.actionBuilder(new Pose2d(37,-43,Math.PI/2))
                .strafeToLinearHeading(new Vector2d(0, -28), 3*Math.PI/2);

        TrajectoryActionBuilder goToGrabSecond = drive.actionBuilder(new Pose2d(0,-25,3*Math.PI/2))
                .strafeToLinearHeading(new Vector2d(37, -60), Math.PI/2);

        TrajectoryActionBuilder goToClipSecond = drive.actionBuilder(new Pose2d(37,-60,Math.PI/2))
                .strafeToLinearHeading(new Vector2d(4, -28), 3*Math.PI/2);

        TrajectoryActionBuilder goToGrabThird = drive.actionBuilder(new Pose2d(4,-25,3*Math.PI/2))
                .strafeToLinearHeading(new Vector2d(37, -60), Math.PI/2);

        TrajectoryActionBuilder goToClipThird = drive.actionBuilder(new Pose2d(37,-60,Math.PI/2))
                .strafeToLinearHeading(new Vector2d(8, -28), 3*Math.PI/2);

        TrajectoryActionBuilder goToGrabFourth = drive.actionBuilder(new Pose2d(8,-25,3*Math.PI/2))
                .strafeToLinearHeading(new Vector2d(37, -60), Math.PI/2);

        TrajectoryActionBuilder goToClipFourth = drive.actionBuilder(new Pose2d(37,-60,Math.PI/2))
                .strafeToLinearHeading(new Vector2d(12, -28), 3*Math.PI/2);

//not gonna spline to samples

//        //
//        TrajectoryActionBuilder moveBackAndSplineToSamples = drive.actionBuilder(new Pose2d(10,-40,drive.getPoseEstimate().heading.toDouble()))
////                .strafeToLinearHeading(new Vector2d(20,-40), 3*Math.PI/2)
//////                .setTangent(Math.PI)
//////                .lineToX(20)
//////                .setTangent(Math.PI/4)
////                .strafeToLinearHeading(new Vector2d(45,-9),3*Math.PI/2);
//
//                .strafeToConstantHeading(new Vector2d(60,-40))
//                .turnTo(new Rotation2d(270.0, 50.0));

//not gonna push the samples to obervation, rather we will use the arm
//
//        TrajectoryActionBuilder pushSamplesToObservationZone = drive.actionBuilder(new Pose2d(45,-9,Math.toRadians(270)))
//                //push first sample down to obervation zone
//                .setTangent(Math.PI/2)
//                .lineToY(-52)
//
//                // go to second sample
//                .lineToY(-12)
//                .setTangent(0)
//                .lineToX(56)
//                .setTangent(Math.PI/2)
//                .lineToY(-52);





        telemetry.update();
        waitForStart();

        if (isStopRequested()) return;

//        SequentialAction testTurning = new SequentialAction(
//                TestTurning.build()
//        );

        ParallelAction GoToClipZero = new ParallelAction(
               // new SequentialAction(
                        slide.slideUp(),
                        forawrdDrive1.build(),
                        //armClaw1.openArmClaw(),
                slideClaw.closeSlideClaw()

                //)


        );

        SequentialAction ClipZero = new SequentialAction(
                slide.slideMidDown(),
                slideClaw.openSlideClaw()
        );

        SequentialAction MoveBack = new SequentialAction(
                moveBack1.build()
        );

        SequentialAction MoveToFirstSampleAndGrabAndDrop = new SequentialAction(
                new ParallelAction(
                        strafeToFirstSample.build(),
                        arm.armReady()
                ),
                arm.armDown(),

                armClaw1.closeArmClaw(),

               new ParallelAction(
                        moveBackToDropFirstSample.build(),
                        arm.armBack()
                ),
                armClaw1.openArmClaw()

        );

        SequentialAction MoveToSecondSampleAndGrabAndDrop = new SequentialAction(
                new ParallelAction(
                        moveToSecondSample.build(),
                        arm.armReady()
                ),
                arm.armDown(),
                armClaw1.closeArmClaw(),
                new ParallelAction(
                        moveBackToDropSecondSample.build(),
                        arm.armBack()
                ),
                armClaw1.openArmClaw()
        );


        SequentialAction MoveToThirdSampleAndGrabAndDrop = new SequentialAction(
                new ParallelAction(
                        moveAndAimAtThirdSample.build(),
                        arm.armReady()
                ),
                arm.armDown(),
                armClaw1.closeArmClaw(),
                new ParallelAction(
                        moveBackToDropThirdSample.build(),
                        arm.armBack()
                ),
                armClaw1.openArmClaw()
        );



        ParallelAction GoToGrabFirst = new ParallelAction(
                goToGrabFirst.build(),
                slide.slideDown(),
                slideClaw.openSlideClaw()
        );

        SequentialAction GrabFirst = new SequentialAction(
                slideClaw.closeSlideClaw()
        );

        ParallelAction GoToClipFirst = new ParallelAction(
                slide.slideUp(),
                goToClipFirst.build()
        );

        SequentialAction ClipFirst = new SequentialAction(
                slide.slideMidDown(),
                slideClaw.openSlideClaw()
        );

        ParallelAction GoToGrabSecond = new ParallelAction(
                goToGrabSecond.build(),
                slide.slideDown(),
                slideClaw.openSlideClaw()
        );

        SequentialAction GrabSecond = new SequentialAction(
                slideClaw.closeSlideClaw()
        );

        ParallelAction GoToClipSecond = new ParallelAction(
                slide.slideUp(),
                goToClipSecond.build()
        );

        SequentialAction ClipSecond = new SequentialAction(
                slide.slideMidDown(),
                slideClaw.openSlideClaw()
        );

        ParallelAction GoToGrabThird = new ParallelAction(
                goToGrabThird.build(),
                slide.slideDown(),
                slideClaw.openSlideClaw()
        );

        SequentialAction GrabThird = new SequentialAction(
                slideClaw.closeSlideClaw()
        );

        ParallelAction GoToClipThird = new ParallelAction(
                goToClipThird.build(),
                slide.slideUp()
        );

        SequentialAction ClipThird = new SequentialAction(
                slide.slideMidDown(),
                slideClaw.openSlideClaw()
        );

        ParallelAction GoToGrabFourth = new ParallelAction(
                goToGrabFourth.build(),
                slide.slideDown(),
                slideClaw.openSlideClaw()
        );

        SequentialAction GrabFourth = new SequentialAction(
                slideClaw.closeSlideClaw()
        );

        ParallelAction GoToClipFourth = new ParallelAction(
                goToClipFourth.build(),
                slide.slideUp()
        );

        SequentialAction ClipFourth = new SequentialAction(
                slide.slideMidDown(),
                slideClaw.openSlideClaw()
        );








        Actions.runBlocking(
                new SequentialAction(
                        GoToClipZero,
                        ClipZero,
                        MoveBack,
                        MoveToFirstSampleAndGrabAndDrop,
                        MoveToSecondSampleAndGrabAndDrop
//                MoveToThirdSampleAndGrabAndDrop,
//                GoToGrabFirst,
//                GrabFirst,
//                GoToClipFirst,
//                ClipFirst,
//                GoToGrabSecond,
//                GrabSecond,
//                GoToClipSecond,
//                ClipSecond,
//                GoToGrabThird,
//                GrabThird,
//                GoToClipThird,
//                ClipThird,
//                GoToGrabFourth,
//                GrabFourth,
//                GoToClipFourth,
//                ClipFourth
                )
    );






    }

}

/*
steps:
- raise arm to to chmberheight while driving forward
- drive back
- spline while loweing thr slide
- push the two samples
- set arm to pickupheight while diagonaling to get ready to pick up
- move to the grabbing position
- close the claw
- strafe to chamber while setting slide to chamber height
- stafe back to pickup location while setting slide to pcik up height
- close claw
- strafe to chamber while setting slide to chamber height
- stafe back to pickup location while setting slide to pcik up height
- close claw
- strafe to chamber while setting slide to chamber height
 */



/*
// step two - move back
                        drive.followTrajectoryAction(
                                drive.actionBuilder(startPose1)
                                        .lineToY(-40)
                                        .build()
                        ),


                        // lower the slide while splining to the three samples
                        new ParallelAction(
                                slide.slideDown(),
                                drive.followTrajectoryAction(
                                        drive.actionBuilder(startPose1)
                                                .setTangent(0)
                                                .lineToX(20)
                                                .splineToConstantHeading(new Vector2d(45, -9), Math.PI / 4)
                                                .build()
                                )
                        ),


                        // Step 4: Push the two samples
                        drive.followTrajectoryAction(
                                drive.actionBuilder(startPose1)
                                        .setTangent(Math.PI / 2)
                                        .lineToY(-52)
                                        // go to second sample
                                        .lineToY(-12)
                                        .setTangent(0)
                                        .lineToX(56)
                                        //push it down to obervation zone
                                        .setTangent(Math.PI/2)
                                        .lineToY(-52)
                                        .build()
                        ),


                        // step 5 - lower the slie while rotating out of the obervation zone
                        // this step is to get ready to p
                        new ParallelAction(
                                slide.slideDown(),
                                drive.followTrajectoryAction(
                                        drive.actionBuilder(startPose1)
                                                .setTangent(-0.25)
                                                .lineToXLinearHeading(38, Math.PI / 2)
                                                .build()
                                )
                        ),


                        // Step 6: Move to grabbing position and close claw
                        new SequentialAction(
                                drive.followTrajectoryAction(
                                        drive.actionBuilder(startPose1)
                                                .setTangent(Math.PI / 2)
                                                .lineToYLinearHeading(-60, Math.PI / 2)
                                                .build()
                                ),
                                slideClaw.closeSlideClaw()
                        ),


                        // Step 7: go to the chamber whilw rasing the slide
                        new ParallelAction(
                                slide.slideUp(),
                                drive.followTrajectoryAction(
                                        drive.actionBuilder(startPose1)
                                                .strafeToLinearHeading(new Vector2d(7, -33.5), 3 * Math.PI / 2)
                                                .build()
                                )
                        ),






                        // Step 8: Return to pickup while lowering slide
                        new ParallelAction(
                                slide.slideDown(),
                                drive.followTrajectoryAction(
                                        drive.actionBuilder(startPose1)
                                                .strafeToLinearHeading(new Vector2d(38, -60), Math.PI / 2)
                                                .build()
                                )
                        ),


                        // Step 9: Repeat grab, chamber, and pickup
                        new SequentialAction(
                                slideClaw.closeSlideClaw(),
                                new ParallelAction(
                                        slide.slideUp(),
                                        drive.followTrajectoryAction(
                                                drive.actionBuilder(startPose1)
                                                        .strafeToLinearHeading(new Vector2d(4, -33.5), 3 * Math.PI / 2)
                                                        .build()
                                        )
                                ),
                                drive.followTrajectoryAction(
                                        drive.actionBuilder(startPose1)
                                                .strafeToLinearHeading(new Vector2d(38, -60), Math.PI / 2)
                                                .build()
                                )
                        ),


                        // Final grab and chamber cycle
                        new SequentialAction(
                                slideClaw.closeSlideClaw(),
                                new ParallelAction(
                                        slide.slideUp(),
                                        drive.followTrajectoryAction(
                                                drive.actionBuilder(startPose1)
                                                        .strafeToLinearHeading(new Vector2d(1, -33.5), 3 * Math.PI / 2)
                                                        .build()
                                        )
                                )
                        )
                )
 */




