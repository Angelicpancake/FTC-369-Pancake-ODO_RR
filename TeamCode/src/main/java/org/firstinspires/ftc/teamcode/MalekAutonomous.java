package org.firstinspires.ftc.teamcode;

// RR-specific imports

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;

// Non-RR imports
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

//import org.firstinspires.ftc.teamcode.teleops.BlueSideTestAuto;

@Config
@Autonomous(name = "Malek auto v2", group = "Autonomous")
public class MalekAutonomous extends LinearOpMode {



    /**
     * The Slide class manages the robot's slide mechanism, including moving the slide up and down.
     */
    public class Slide {
        private DcMotor slide;

        public Slide(HardwareMap hardwareMap) {
            slide = hardwareMap.get(DcMotor.class, "slide");
        }

        /**
         * The SlideUp class represents the action of moving the slide upward.
         */
        public class SlideUp implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (!initialized) {
                    slide.setPower(0.8);
                    initialized = true;
                }

                double pos = slide.getCurrentPosition();
                telemetryPacket.put("SlidePosition", pos);
                if (pos < 3000.0) {
                    return true;
                } else {
                    slide.setPower(0.01);
                    return false;
                }
            }
        }

        public Action slideUp() {
            return new SlideUp();
        }

        /**
         * The SlideDown class represents the action of moving the slide downward.
         */
        public class SlideDown implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    slide.setPower(-0.8);
                    initialized = true;
                }

                double pos = slide.getCurrentPosition();
                packet.put("SlidePosition", pos);
                if (pos > 100.0) {
                    return true;
                } else {
                    slide.setPower(0.01);
                    return false;
                }
            }
        }

        public Action slideDown() {
            return new SlideDown();
        }
    }

    /**
     * The SlideClaw class manages the robot's slide claw mechanism, including opening and closing the claw.
     */
    public class SlideClaw {
        private Servo slideClaw;

        public SlideClaw(HardwareMap hardwareMap) {
            slideClaw = hardwareMap.get(Servo.class, "slideClaw");
        }

        /**
         * The CloseSlideClaw class represents the action of closing the slide claw.
         */
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

        /**
         * The OpenSlideClaw class represents the action of opening the slide claw.
         */
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

    /**
     * The ArmClaw class manages the robot's arm claw mechanism, including opening and closing the claw.
     */
    public class ArmClaw {
        private Servo armClaw;

        public ArmClaw(HardwareMap hardwareMap) {
            armClaw = hardwareMap.get(Servo.class, "armClaw");
        }

        /**
         * The CloseArmClaw class represents the action of closing the arm claw.
         */
        public class CloseArmClaw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                armClaw.setPosition(0.75);
                return false;
            }
        }

        public Action closeArmClaw() {
            return new CloseArmClaw();
        }

        /**
         * The OpenArmClaw class represents the action of opening the arm claw.
         */
        public class OpenArmClaw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                armClaw.setPosition(0.1);
                return false;
            }
        }

        public Action openArmClaw() {
            return new OpenArmClaw();
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize the robot's mechanisms
        Pose2d initialPose = new Pose2d(10, -60, 3*Math.PI/2);
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        ArmClaw claw = new ArmClaw(hardwareMap);
        SlideClaw slideClaw =  new SlideClaw(hardwareMap);


        // Add your autonomous logic here, using the actions provided by the inner classes
    }
}