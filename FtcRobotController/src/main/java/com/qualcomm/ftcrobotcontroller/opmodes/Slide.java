package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

public class Slide extends OpMode {

    final static double BOX_MIN_RANGE  = 0.2;
    final static double BOX_MAX_RANGE  = 0.9;



    double boxPosition;

    double boxDelta = 0.01;




//Drive Train

    DcMotor motor_1;
    DcMotor motor_2;

    Servo box;






    public Slide(){

    }
    @Override
    public void init() {
        //renaming drivetrain


        //renaming slide

        motor_1 = hardwareMap.dcMotor.get("motor_1");
        motor_2= hardwareMap.dcMotor.get("motor_2");
        motor_2.setDirection(DcMotor.Direction.REVERSE);

        box = hardwareMap.servo.get("servo_1");
        boxPosition= 0.2;



    }


    @Override
    public void loop() {


       //slide in and out

        if (gamepad1.a) {
            motor_1.setPower(-1);
            motor_2.setPower(1);

        }


       else if (gamepad1.b) {
            motor_1.setPower(1);
            motor_2.setPower(-1);
        }
        else {
            motor_1.setPower(0);
            motor_2.setPower(0);

        }

        if(gamepad1.x){
            boxPosition += boxDelta;

        }
        if (gamepad1.y){
            boxPosition -= boxDelta;

        }
        boxPosition = Range.clip(boxPosition, BOX_MIN_RANGE, BOX_MAX_RANGE);

        // write position values to the wrist and claw servo
        box.setPosition(boxPosition);








		/*
		 * Send telemetry data back to driver station. Note that if we are using
		 * a legacy NXT-compatible motor controller, then the getPower() method
		 * will return a null value. The legacy NXT-compatible motor controllers
		 * are currently write only.
		 */
        telemetry.addData("Text", "*** Robot Data***");

    }



        }
