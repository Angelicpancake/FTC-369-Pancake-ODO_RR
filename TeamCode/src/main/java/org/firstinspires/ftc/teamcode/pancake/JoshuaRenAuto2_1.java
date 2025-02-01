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

            specimenClaw.setPosition(0.8);//start SpecimenClaw closed
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

                if (isReset && timer.seconds() > 0.5) return false;//exit after reset

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

                if (timer.seconds() > 0.5){isReset = false;
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
        Pose2d initialPose = new Pose2d(10, -62, Math.toRadians(90));

        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose); //initialize drive
        BridgeArmClaw bac = new BridgeArmClaw(hardwareMap);//initial all servos

        //Trajectories
        TrajectoryActionBuilder placeSample0T = drive.actionBuilder(initialPose)
                .strafeToConstantHeading(new Vector2d(-1, -31));

        TrajectoryActionBuilder pushT = placeSample0T.endTrajectory().fresh()
                .strafeToConstantHeading(new Vector2d(26, -42))
                .splineToConstantHeading(new Vector2d(43, -12), Math.toRadians(30))
                .strafeToConstantHeading(new Vector2d(43, -50))
                .splineToConstantHeading(new Vector2d(53, -13), Math.toRadians(10))
                .strafeToConstantHeading(new Vector2d(53,-50))
                .splineToConstantHeading(new Vector2d(63, -13), Math.toRadians(10))
                .strafeToConstantHeading(new Vector2d(63, -50));

        TrajectoryActionBuilder specimen1T = pushT.endTrajectory().fresh()
                .strafeToConstantHeading(new Vector2d(38, -65))
                .strafeToConstantHeading(new Vector2d(-4, -31));

        TrajectoryActionBuilder specimen2T = specimen1T.endTrajectory().fresh()
                .strafeToConstantHeading(new Vector2d(38, -65))
                .strafeToConstantHeading(new Vector2d(-2, -31));

        TrajectoryActionBuilder specimen3T = specimen2T.endTrajectory().fresh()
                .strafeToConstantHeading(new Vector2d(38, -65))
                .strafeToConstantHeading(new Vector2d(0, -31));

        TrajectoryActionBuilder specimen4T = specimen3T.endTrajectory().fresh()

                .strafeToConstantHeading(new Vector2d(38, -65))
                .strafeToConstantHeading(new Vector2d(1, -31));

        TrajectoryActionBuilder parkT = specimen4T.endTrajectory().fresh()
                .strafeToConstantHeading(new Vector2d(38,-65));

        waitForStart();//wait until start is pressed

        if (isStopRequested()) return; //exit when stop is pressed

        //Sequential Actions + Parallel

        /*hang specimen0 (starting specimen)
          1. extend rail and swing specimenArm
          2. drive forward
          3. open claw */
        SequentialAction hangSpecimen0 = new SequentialAction(
                new ParallelAction(
                        bac.hangSpecimen(),
                        placeSample0T.build()
                )
        );

        /* push samples (3 samples)
            1. reset rail and swing arm back
            2. do the driving path
         */
        SequentialAction pushSamples = new SequentialAction(
                new ParallelAction(
                        bac.resetRailArm(),
                        pushT.build()
                )
        );

        /* Clip Specimen #1 (pre-wall specimen) (repeat 1-4)
            1. close claw (claw should already be opened)
            2. go to hang specimen then come back
         */

        SequentialAction hangSpecimen1 = new SequentialAction(
                bac.closeSpecimenClaw(),
                specimen1T.build()
        );
        SequentialAction hangSpecimen2 = new SequentialAction(
                bac.closeSpecimenClaw(),
                specimen2T.build()
        );
        SequentialAction hangSpecimen3 = new SequentialAction(
                bac.closeSpecimenClaw(),
                specimen3T.build()
        );
        SequentialAction hangSpecimen4 = new SequentialAction(
                bac.closeSpecimenClaw(),
                specimen4T.build()
        );

        // Park

        SequentialAction park = new SequentialAction(
            parkT.build()
        );



        //run actions
        Actions.runBlocking(
                new SequentialAction(
                        hangSpecimen0,
                        pushSamples,
                        hangSpecimen1,
                        hangSpecimen2,
                        hangSpecimen3,
                        hangSpecimen4,
                        park
                )
        );

    }
}
