package org.firstinspires.ftc.teamcode.pancake;

//roadrunner imports
import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;

//ftc imports
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.MecanumDrive;

import dalvik.system.DelegateLastClassLoader;

@Autonomous(name = "PancakeRight-Auto", group = "Autonomous")
public class JoshuaRenAuto2_1 extends LinearOpMode {
    public class BridgeArmClaw {
        private Servo bridge, sampleClaw, specimenArm, specimenClaw, rail;
        private ElapsedTime timer = new ElapsedTime();
        //multi declaration of Servo variables

        //Pre Cond: initialize the Servo Vars through HardwareMap
        public BridgeArmClaw (HardwareMap hardwareMap){
            bridge = hardwareMap.get(Servo.class, "bridge");
            sampleClaw = hardwareMap.get(Servo.class, "sampleClaw");
            specimenArm = hardwareMap.get(Servo.class, "specimenArm");
            specimenClaw = hardwareMap.get(Servo.class, "specimenClaw");
            rail = hardwareMap.get(Servo.class, "rail");

            specimenClaw.setPosition(0);//start SpecimenClaw open
            rail.setPosition(0);//rail set down
        }//Constructor

        public class CloseSpecimenClaw implements Action{
            private boolean isReset = false;

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (!isReset){
                    isReset = true;
                    timer.reset();
                }
                specimenClaw.setPosition(0.8);

                if (timer.seconds() > 0.5) return false;

                return true;
            }
        }//CloseSpecimenArm

        public class ResetRailArm implements Action {
            private boolean isReset = false;
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (!isReset){
                    specimenArm.setPosition(1);//set to reset value (back)
                    rail.setPosition(0); //set to reset value (down)
                    timer.reset();
                    isReset = true;
                }

                if (timer.seconds() > 0.5) return false;//exit after reset

                return true;
            }
        }//ResetRailArm

        //This action will be used to hang specimen in the sequential actions
        public class HangSpecimen implements Action {
            private boolean isReset = false;
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (!isReset){
                    isReset = true;
                    timer.reset();
                }
                /*rail should go up, swing specimenArm to position
                rail => 0.5; sA => 0*/
                rail.setPosition(0.5);
                specimenArm.setPosition(0);

                //change these positions if needed

                if (timer.seconds() > 0.7){isReset = false;
                    specimenClaw.setPosition(0);//open SpecimenClaw
                    return false;
                }
                return true;//false exits
            }
        }//hangSpecimen

        //Declare Actions

        public Action closeSpecimenClaw(){
            return new CloseSpecimenClaw();
        }

        public Action hangSpecimen() {
            return new HangSpecimen();
        }

        public Action resetRailArm(){
            return new ResetRailArm();
        }


    }//BridgeArmClaw

    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d initialPose = new Pose2d(25, -62, Math.toRadians(90));

        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose); //initialize drive
        BridgeArmClaw bac = new BridgeArmClaw(hardwareMap);//initial all servos

        //Trajectories
        TrajectoryActionBuilder pushSamplesTraj = drive.actionBuilder(initialPose)
                .splineToConstantHeading(new Vector2d(40, -13), Math.toRadians(73))
                .strafeToConstantHeading(new Vector2d(43, -50))
                .splineToConstantHeading(new Vector2d(53, -13), Math.toRadians(10))
                .strafeToConstantHeading(new Vector2d(53,-50))
                .splineToConstantHeading(new Vector2d(61, -13), Math.toRadians(10))
                .strafeToConstantHeading(new Vector2d(61, -50))
                .splineToConstantHeading(new Vector2d(35, -62), Math.toRadians(240));

        TrajectoryActionBuilder specimen1Traj = pushSamplesTraj.endTrajectory().fresh()
                //  .strafeToConstantHeading(new Vector2d(38, -65))
                .strafeToConstantHeading(new Vector2d(-5, -31));

        TrajectoryActionBuilder specimen2Traj = specimen1Traj.endTrajectory().fresh()
                .strafeToConstantHeading(new Vector2d(35, -62))
                .strafeToConstantHeading(new Vector2d(-4, -31));

        TrajectoryActionBuilder specimen3Traj = specimen2Traj.endTrajectory().fresh()
                .strafeToConstantHeading(new Vector2d(35, -62))
                .strafeToConstantHeading(new Vector2d(-2, -31));

        TrajectoryActionBuilder specimen4Traj = specimen3Traj.endTrajectory().fresh()
                .strafeToConstantHeading(new Vector2d(35, -62))
                .strafeToConstantHeading(new Vector2d(0, -31));

        TrajectoryActionBuilder specimen5Traj = specimen4Traj.endTrajectory().fresh()
                .strafeToConstantHeading(new Vector2d(35, -62))
                .strafeToConstantHeading(new Vector2d(1, -31));

        TrajectoryActionBuilder parkTraj = specimen4Traj.endTrajectory().fresh()
                .strafeToConstantHeading(new Vector2d(38,-65));

        waitForStart();//wait until start is pressed

        if (isStopRequested()) return; //exit when stop is pressed

        //Sequential Actions + Parallel

        /* push samples (3 samples)
            1. reset rail and swing arm back
            2. do the driving path
         */
        SequentialAction pushSamples = new SequentialAction(
                new ParallelAction(
                        pushSamplesTraj.build()
                )
        );

        /* Clip Specimen #1 (pre-wall specimen) (repeat 1-5)
            1. close claw (claw should already be opened)
            2. go to hang specimen then come back
         */

        SequentialAction hangSpecimen1 = new SequentialAction(
                bac.closeSpecimenClaw(),
                new ParallelAction() (
                    specimen1Traj.build()
                    bac.hangSpecimen()
                )

        );
        SequentialAction hangSpecimen2 = new SequentialAction(
                bac.closeSpecimenClaw(),
                new ParallelAction()(
                    specimen2Traj.build()
                    bac.hangSpecimen()
                )
        );
        SequentialAction hangSpecimen3 = new SequentialAction(
                bac.closeSpecimenClaw(),
                new ParallelAction()(
                    specimen3Traj.build()
                    bac.hangSpecimen()
                )
        );
        SequentialAction hangSpecimen4 = new SequentialAction(
                bac.closeSpecimenClaw(),
                new ParallelAction()(
                    specimen4Traj.build()
                    bac.hangSpecimen()
                )
        );

        SequentialAction hangSpecimen5 = new SequentialAction(
                bac.closeSpecimenClaw(),
                new ParallelAction()(
                        specimen5Traj.build()
                        bac.hangSpecimen()
                )
        );

        // Park

        SequentialAction park = new SequentialAction(
            parkTraj.build()
        );



        //run actions
        Actions.runBlocking(
                new SequentialAction(
                        pushSamples,
                        hangSpecimen1,
                        hangSpecimen2,
                        hangSpecimen3,
                        hangSpecimen4,
                        hangSpecimen5,
                        park
                )
        );

    }
}
