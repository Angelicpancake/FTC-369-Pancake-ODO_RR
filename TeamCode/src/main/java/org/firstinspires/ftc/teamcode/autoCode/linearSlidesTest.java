package org.firstinspires.ftc.teamcode.autoCode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="WIP2 Linear Slide")
public class linearSlidesTest extends OpMode {
    //op mode -> real time control, linearOp -> preset commands (auto)
    private DcMotor linearSlide1;
    public ElapsedTime runtime;

    @Override
    public void init(){
        linearSlide1 = hardwareMap.get(DcMotor.class, "slide");
        linearSlide1.setDirection(DcMotor.Direction.REVERSE); //how it should determine positive/negative positions
        runtime = new ElapsedTime();
        runtime.reset(); //set the counter for runtime to be 0
    }

    @Override
    public void loop() {
        double currentPosition = 0.0;

        telemetry.addData("Runtime", runtime.seconds());
        telemetry.addData("Position", linearSlide1.getCurrentPosition() );
        telemetry.update();
        //print out the data of the linear slide position on the game hub

        //have to set the power depending on the tests

        double power = 0.5;


        // remember it is reversed -> init


        //using a variable for true or false for a and b



        //up
        if(gamepad2.right_trigger>0 && linearSlide1.getCurrentPosition()<3000){
            linearSlide1.setPower(gamepad2.right_trigger*power);
        }
        else if(gamepad2.left_trigger>0.3){
            linearSlide1.setPower(gamepad2.left_trigger*-1*power);
        }
        else {
            linearSlide1.setPower(0.05);
        }
    }
}