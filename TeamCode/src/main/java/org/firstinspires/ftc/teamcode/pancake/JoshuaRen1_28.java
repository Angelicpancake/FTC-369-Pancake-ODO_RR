package org.firstinspires.ftc.teamcode.pancake;
/////p
import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Chesapeake Teleop")
public class JoshuaRen1_28 extends OpMode {
    private DcMotor backLeft;
    private DcMotor backRight;
    private DcMotor frontLeft;
    private DcMotor frontRight;

    public Servo bridge;
    public Servo sampleClaw;
    public Servo specimenArm;
    public Servo specimenClaw;
    public Servo rail;

    private IMU imu;
    public double DRIVE_POWER_VARIABLE = 1;
    public boolean ss;

    private ElapsedTime bridgeTimer = new ElapsedTime();




    @Override
    public void init() {
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");


        backRight.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.REVERSE);

        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        imu = hardwareMap.get(IMU.class, "imu");
        //initializing the imu
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        //adding the orientation of the imu for accurate calculations
        imu.initialize(parameters);

        // Reset IMU heading
        imu.resetYaw();

        /* servo initialize*/
        bridge = hardwareMap.get(Servo.class, "bridge");
        sampleClaw = hardwareMap.get(Servo.class, "sampleClaw");
        specimenArm = hardwareMap.get(Servo.class, "specimenArm");
        specimenClaw = hardwareMap.get(Servo.class, "specimenClaw");
        rail = hardwareMap.get(Servo.class, "rail");


        this.ss = true;
        this.rail.setPosition(0);
        this.specimenClaw.setPosition(0);
    }

    @Override
    public void loop() {

        if (gamepad2.dpad_up) //preset //sometimes the boolean ss dosen't change when pressing dpad
        {   //starts at !armExtended
            //this.ss = false;
            this.rail.setPosition(0.45);
            this.specimenArm.setPosition(0.15);
        }
        else if (gamepad2.dpad_right)
        {
            //this.ss = true;
            this.rail.setPosition(0);
            this.specimenArm.setPosition(0.85);

        }
       // else if (gamepad2.dpad_left)

        // Get drive inputs (negated Y because joystick Y is reversed)
        double x = -gamepad1.left_stick_x;
        double y = gamepad1.left_stick_y;  // Negated to match standard coordinate system
        double rx = -DRIVE_POWER_VARIABLE * (gamepad1.right_trigger - gamepad1.left_trigger);
        //gamepad1.right_stick_x * 0.85;


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

        // gamepad1 - movement systems (Malek)
        if (gamepad1.left_bumper) DRIVE_POWER_VARIABLE = 0.5; // slow driving
        else if (gamepad1.right_bumper) DRIVE_POWER_VARIABLE = 1; // revert to normal speed (fast)
        else if (gamepad1.b) sampleClaw.setPosition(0); // opens sample claw
        else if (gamepad1.y) sampleClaw.setPosition(0.8); // closes sample claw
        else if (gamepad1.x) imu.resetYaw();

        // Debug telemetry
        telemetry.addData("railPosition", rail.getPosition());
        telemetry.addData("Robot Heading", Math.toDegrees(botHeading));
        telemetry.addData("left stick y", y);
        telemetry.addData("left stick x ", x);
        telemetry.addData("booleanValue", ss);
        telemetry.addData("armPos", specimenArm.getPosition());
        telemetry.update();

        // gamepad2 - intake systems (Kaichen)
        if (gamepad2.left_bumper) specimenClaw.setPosition(0);
        else if (gamepad2.right_bumper) specimenClaw.setPosition(1.0);
        else if (gamepad2.left_trigger > 0.1) sampleClaw.setPosition(0); // opens specimen claw
        else if (gamepad2.right_trigger > 0.1) sampleClaw.setPosition(0.8); // closes specimen claw
        else if (gamepad2.a) bridge.setPosition(0); // drops sample arm all the way down
        else if (gamepad2.y) bridge.setPosition(1); // raises sample arm
        else if (gamepad2.b) bridge.setPosition(0.4); // "hover" mode
        else if (gamepad2.x) bridge.setPosition(0.5); // "backout" mode
        else if (gamepad2.dpad_left) bridge.setPosition(0.6);//back up specimen score
       /* else if (gamepad2.x) {
            bridgeTimer.reset();

            bridge.setPosition(0);
            if (bridge.getPosition()== 0 && bridgeTimer.milliseconds() >= 500){
                sampleClaw.setPosition(0.8);
                bridge.setPosition(0.4);
            }
        }*/
    }
}

