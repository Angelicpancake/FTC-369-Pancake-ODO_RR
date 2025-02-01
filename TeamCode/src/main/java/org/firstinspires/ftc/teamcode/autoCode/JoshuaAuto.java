package org.firstinspires.ftc.teamcode.autoCode;

// RR-specific imports

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;


//non rr imports
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.MecanumDrive;


@Config
@Autonomous(name = "Joshua Autonomous 1/15", group = "Autonomous")
public class JoshuaAuto extends LinearOpMode {

    private int chamberHeight = 2115;
    private int clipOnChamberHeight = 790;
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
                slide.setPower(1);

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
        ElapsedTime timer1 = new ElapsedTime();

        public SlideClaw(HardwareMap hardwareMap) {
            slideClaw = hardwareMap.get(Servo.class, "slideClaw");
        }

        public class CloseSlideClaw implements Action {
            private  boolean isReset = false;
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!isReset) {
                    isReset = true;
                    timer1.reset();
                }
                slideClaw.setPosition(1);


                if(timer1.seconds() > 0.5) {
                    return false;
                }

                return true;
            }
        }
        public Action closeSlideClaw() {
            return new CloseSlideClaw();
        }

        public class OpenSlideClaw implements Action {
            private  boolean isReset = false;
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!isReset) {
                    isReset = true;
                    timer1.reset();
                }
                slideClaw.setPosition(0.1);


                if(timer1.seconds() > 0.5) {
                    return false;
                }

                return true;
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
                arm.setPower(1);

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
                arm.setPower(1);

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
                arm.setPower(1);

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
        Pose2d startPose1 = new Pose2d(10, -62, Math.toRadians(270));
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose1);
        Slide slide = new Slide(hardwareMap);
        SlideClaw slideClaw = new SlideClaw(hardwareMap);
        Arm arm = new Arm(hardwareMap);
        ArmClaw armClaw1 = new ArmClaw(hardwareMap);

        /*
            The Autonomous
         */

        //specimen #1
        TrajectoryActionBuilder forwardDrive1 = drive.actionBuilder(startPose1)
                .strafeTo(new Vector2d(-4,-20)); //go to specimen

        TrajectoryActionBuilder moveBack1 = forwardDrive1.endTrajectory().fresh()
            .lineToY(-40); //move away from hang

        //push pieces
        TrajectoryActionBuilder pushing = moveBack1.endTrajectory().fresh()
                .strafeToConstantHeading(new Vector2d(54,-40))// move right

                .strafeToConstantHeading(new Vector2d(54,1))// move forward

                .strafeToConstantHeading(new Vector2d(70,0))// move right and a bit back

                .strafeToConstantHeading(new Vector2d(70,-45), new TranslationalVelConstraint(400))// push the first specimen down

                .strafeToConstantHeading(new Vector2d(70,1), new TranslationalVelConstraint(400))// go back up

                .strafeToConstantHeading(new Vector2d(86,0))// align to the second sample

                .strafeToConstantHeading(new Vector2d(86,-45), new TranslationalVelConstraint(400)); // push the second down

        //grab specimen wall#1
        TrajectoryActionBuilder goToGrabFirst = pushing.endTrajectory().fresh()
                .turn(Math.toRadians(270)); //change it to go 270 first then move down

        //clip specimen wall#1
        TrajectoryActionBuilder goToClipFirst = goToGrabFirst.endTrajectory().fresh()
                .strafeToConstantHeading(new Vector2d(10,-28))
                .strafeToConstantHeading(new Vector2d(14,-13));

        //grab specimen wall#2
        TrajectoryActionBuilder goToGrabSecond = drive.actionBuilder(new Pose2d(0,-25,3*Math.PI/2))
                .setTangent(Math.toRadians(90))
                .strafeToLinearHeading(new Vector2d(60, -65), Math.toRadians(90))
                .strafeToConstantHeading(new Vector2d(63, -69));//


        TrajectoryActionBuilder goToClipSecond = drive.actionBuilder(new Pose2d(37,-60,Math.PI/2))
                .strafeToLinearHeading(new Vector2d(6, -49), Math.toRadians(-270))
                .strafeToConstantHeading(new Vector2d(6, -28)); // too far back

        TrajectoryActionBuilder goToGrabThird = drive.actionBuilder(new Pose2d(4,-25,3*Math.PI/2))
                .strafeToLinearHeading(new Vector2d(52, -70), Math.toRadians(95))
                .strafeToConstantHeading(new Vector2d(52, -91)); //

        TrajectoryActionBuilder goToClipThird = drive.actionBuilder(new Pose2d(37,-60,Math.PI/2))
                .strafeToLinearHeading(new Vector2d(0, -50), Math.toRadians(270))
                .strafeToConstantHeading(new Vector2d(1, -27));//


//        TrajectoryActionBuilder goToGrabFourth = drive.actionBuilder(new Pose2d(8,-25,3*Math.PI/2))
//                .strafeToLinearHeading(new Vector2d(37, -60), Math.PI/2);
//
//        TrajectoryActionBuilder goToClipFourth = drive.actionBuilder(new Pose2d(37,-60,Math.PI/2))
//                .strafeToLinearHeading(new Vector2d(12, -30), 3*Math.PI/2);

        TrajectoryActionBuilder GoToPark  = drive.actionBuilder(new Pose2d(12, -30, 3*Math.PI/2))
                .strafeToLinearHeading(new Vector2d(80,-80), Math.toRadians(200));

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
                forwardDrive1.build(),
                //armClaw1.openArmClaw(),
                slideClaw.closeSlideClaw()

                //)


        );

        SequentialAction ClipZero = new SequentialAction(
                slide.slideMidDown(),
                slideClaw.openSlideClaw()
        );

//        SequentialAction MoveBack = new SequentialAction(
//                moveBack1.build()
//        );

        SequentialAction pushFirstTwoSamples = new SequentialAction(
                pushing.build()

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

//        ParallelAction GoToGrabFourth = new ParallelAction(
//                goToGrabFourth.build(),
//                slide.slideDown(),
//                slideClaw.openSlideClaw()
//        );
//
//        SequentialAction GrabFourth = new SequentialAction(
//                slideClaw.closeSlideClaw()
//        );
//
//        ParallelAction GoToClipFourth = new ParallelAction(
//                goToClipFourth.build(),
//                slide.slideUp()
//        );

        SequentialAction ClipFourth = new SequentialAction(
                slide.slideMidDown(),
                slideClaw.openSlideClaw()
        );

        ParallelAction Park = new ParallelAction(
                GoToPark.build(),
                arm.armReady()
        );








        Actions.runBlocking(
                new SequentialAction(
                        GoToClipZero,
                        ClipZero,
                        pushFirstTwoSamples,
                        GoToGrabFirst,
                        GrabFirst,
                        GoToClipFirst,
                        ClipFirst,
                        GoToGrabSecond,
                        GrabSecond,
                        GoToClipSecond,
                        ClipSecond,
                        GoToGrabThird,
                        GrabThird,
                        GoToClipThird,
                        ClipThird
                        //Park
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



