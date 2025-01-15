package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name="369 FINAL TELE-OP COMP CODE")
public class FinalCombinedFCC extends OpMode {
    private DcMotor backLeft;
    private DcMotor backRight;
    private DcMotor frontLeft;
    private DcMotor frontRight;

    private ElapsedTime timer;

    private double DRIVE_POWER_VARIABLE = 0.92;
    private double pow = 0.7;
    private boolean SLOW_MODE = false;


    public Servo wrist;

    private IMU imu;

    // Linear slide motor + claw
    private DcMotor slide = null;
    private Servo slideClaw = null;


    //  arm motor  and claw servo
    private DcMotor arm = null;

    private Servo armClaw = null;

    //private Servo angleClaw = null;

    private ElapsedTime runtime = new ElapsedTime();

    // Arm position constants
    private static final double ARM_POWER_SCALE = 0.6;
    private static final int ARM_LOWER_LIMIT = -5000;
    private static final int ARM_UPPER_LIMIT = 7000;

    // Claw position constants
    private static final double CLAW_OPEN_POSITION = 0.1;
    private static final double CLAW_CLOSED_POSITION = 0.75;

    private int chamberHeight = 2000;
    private int clipOnChamberHeight = 750;
    private int grabFromWallHeight = 100;

    @Override
    public void init() {
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");

        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontLeft.setDirection(DcMotor.Direction.REVERSE);

        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        wrist = hardwareMap.get(Servo.class, "wristServo");




        // Initialize linear slide motor and claw
        slide = hardwareMap.get(DcMotor.class, "slide");
        slide.setDirection(DcMotor.Direction.REVERSE);
        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        slideClaw = hardwareMap.get(Servo.class, "slideClaw");
        timer = new ElapsedTime();

        // Initialize arm motor
        arm = hardwareMap.get(DcMotor.class, "arm");
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        armClaw = hardwareMap.get(Servo.class, "armClaw");



        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD));
        imu.initialize(parameters);

        // Reset IMU heading
        imu.resetYaw();

        telemetry.addLine("Hardware Initialized");
        telemetry.update();
    }

    @Override
    public void loop() {
        // Get drive inputs (negated Y because joystick Y is reversed)
        double x = gamepad1.left_stick_x;
        double y = -gamepad1.left_stick_y;  // Negated to match standard coordinate system
        double rx = DRIVE_POWER_VARIABLE*(gamepad1.right_trigger - gamepad1.left_trigger);
        //gamepad1.right_stick_x * 0.85;

        // Get the robot's heading
        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        // Apply field centric transformation
        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        // Calculate motor powers using transformed inputs
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        double frontLeftPower = (rotY + rotX + rx) / denominator;
        double frontRightPower = (rotY - rotX - rx) / denominator;
        double backLeftPower = (rotY - rotX + rx) / denominator;
        double backRightPower = (rotY + rotX - rx) / denominator;

        // Set motor powers
        frontLeft.setPower(frontLeftPower * DRIVE_POWER_VARIABLE);
        frontRight.setPower(frontRightPower * DRIVE_POWER_VARIABLE);
        backLeft.setPower(backLeftPower * DRIVE_POWER_VARIABLE);
        backRight.setPower(backRightPower * DRIVE_POWER_VARIABLE);


        handleArmControl();


        handleArmClawControl();


        handleLinearSlideControl();

        handleSlideClawControl();

        handleWrist();

        handleSlowDrivingMode();

        if(gamepad1.x){
            imu.resetYaw();
        }

        // try to fix this feature if we have time
//        if(gamepad1.dpad_up){
//            if (SLOW_MODE){
//                DRIVE_POWER_VARIABLE = 1;
//            } else if (!SLOW_MODE) {
//                DRIVE_POWER_VARIABLE = 0.5;
//            }
//            SLOW_MODE=!SLOW_MODE;
//
//        };






        // Debug telemetry
        telemetry.addData("Robot Heading", Math.toDegrees(botHeading));
        telemetry.addData("Slide Position", slide.getCurrentPosition());
        telemetry.addData("Drive Data", "X (%.2f), Y (%.2f), RX (%.2f)", x, y, rx);
        telemetry.addData("Transformed", "rotX (%.2f), rotY (%.2f)", rotX, rotY);
        telemetry.update();
    }

    private void handleArmControl() {
        int currentPosition = arm.getCurrentPosition();


        double armPower = -gamepad2.left_stick_y * ARM_POWER_SCALE;

        // Add position limits
        if ((currentPosition >= ARM_UPPER_LIMIT && armPower > 0) ||
                (currentPosition <= ARM_LOWER_LIMIT && armPower < 0)) {
            armPower = 0;
        } else if (gamepad2.b) {
            armPower = 0.05;
        }

        arm.setPower(armPower);
    }

    private void handleArmClawControl() {
        // Open claw with left trigger, close with right trigger on gamepad2
        if (gamepad2.right_trigger > 0.3) {
            armClaw.setPosition(0.8);  // Closed position
        } else if (gamepad2.left_trigger > 0.3) {
            armClaw.setPosition(0.2);  // Open position
        } else if (gamepad1.right_bumper) {
            double tempArmClawPos = armClaw.getPosition();
            if (tempArmClawPos > 0.4){
                armClaw.setPosition(0.2);
            } else if (tempArmClawPos < 0.4) {
                armClaw.setPosition(0.8);

            }
        }
    }

    private void handleSlideClawControl() {
        // Open claw with left bumper, close with right bumper on gamepad2
        if (gamepad2.right_bumper) {
            slideClaw.setPosition(0.8);  // Closed position
        } else if (gamepad2.left_bumper) {
            slideClaw.setPosition(0.1);  // Open position
        }

    }

    private void handleSlowDrivingMode(){



        if (gamepad1.dpad_up){
            frontLeft.setPower(pow);
            frontRight.setPower(pow);
            backLeft.setPower(pow);
            backRight.setPower(pow);
        } else if (gamepad1.dpad_down) {
            frontLeft.setPower(-pow);
            frontRight.setPower(-pow);
            backLeft.setPower(-pow);
            backRight.setPower(-pow);
        } else if (gamepad1.dpad_right) {
            frontLeft.setPower(pow);
            frontRight.setPower(-pow);
            backLeft.setPower(-pow);
            backRight.setPower(pow);
        } else if (gamepad1.dpad_left){
            frontLeft.setPower(-pow);
            backLeft.setPower(pow);

            frontRight.setPower(pow);
            backRight.setPower(-pow);
        }
    }

    private void handleWrist(){
        if(gamepad2.dpad_up)
            wrist.setPosition(1);
        else if (gamepad2.dpad_left) {
            wrist.setPosition(0.35);
        } else if(gamepad2.dpad_down)
            wrist.setPosition(0);
        else if(gamepad2.dpad_right) { //preset
            timer.reset();
            wrist.setPosition(0);
            while (timer.seconds() < 1.5)
                arm.setPower(0.4);

            arm.setPower(0);

        }
    }

    private void handleLinearSlideControl() {
        // Use right joystick for linear slide control
        double power = 1;
        int currentPosition = slide.getCurrentPosition();

        // Linear slide control using right joystick on gamepad2
        double slidePower = -gamepad2.right_stick_y;
        if (Math.abs(slidePower) > 0.15){
            slide.setPower(slidePower);
        }



        // old complicated cpmtrl for slide, taken out becasue slide wont end at the bottom in auto
//        if (gamepad2.y){
//            slide.setTargetPosition(chamberHeight);
//            slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            slide.setPower(1.0);
//        } else if (gamepad2.a){
//            slide.setTargetPosition(grabFromWallHeight);
//            slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            slide.setPower(1.0);
//        } else if (Math.abs(slidePower) > 0.2) {
//            slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            // Check upper and lower limits
//            if ((currentPosition >= 2750 && slidePower > 0) ||
//                    (currentPosition <= 3 && slidePower < 0)) {
//                slide.setPower(0.01); // Hold position
//            } else {
//                slide.setPower(slidePower * power);
//            }
//        } else {
//            slide.setPower(0);
//
//        }


//        if (gamepad2.y){
//            slide.setTargetPosition(chamberHeight);
//            slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            slidePower = 1.0;
//        } else if (gamepad2.a){
//            slide.setTargetPosition(grabFromWallHeight);
//            slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            slidePower = 1.0;
//        } else if (Math.abs(slidePower) > 0.2) {
//            slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            if(slidePower>0.2){
//                slide.setTargetPosition(chamberHeight);
//                slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            } else if (slidePower < -0.2){
//                slide.setTargetPosition(grabFromWallHeight);
//                slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            }
//        } else {
//            slide.setPower(slidePower);
//            if(slide.getCurrentPosition()==slide.getTargetPosition())
//                slidePower = -gamepad1.left_stick_y;
//
//        }
    }
}