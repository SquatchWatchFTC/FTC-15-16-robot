package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

public class Drive extends OpMode {



//Drive Train
    DcMotor leftMotor;
    DcMotor rightMotor;







    public Drive(){

    }
    @Override
    public void init() {
        //renaming drivetrain

        rightMotor = hardwareMap.dcMotor.get("right");
        leftMotor = hardwareMap.dcMotor.get("left");
        rightMotor.setDirection(DcMotor.Direction.REVERSE);//CHANGED RIGHT




    }


    @Override
    public void loop() {

        float leftY = -gamepad1.left_stick_y;
        float rightY = - gamepad1.right_stick_y;


        // write the values to the motors
        leftMotor.setPower(leftY);
        rightMotor.setPower(rightY);

















		/*
		 * Send telemetry data back to driver station. Note that if we are using
		 * a legacy NXT-compatible motor controller, then the getPower() method
		 * will return a null value. The legacy NXT-compatible motor controllers
		 * are currently write only.
		 */
        telemetry.addData("Text", "*** Robot Data***");
        telemetry.addData("left tgt pwr", "Left  pwr: " + String.format("%.2f", leftY));
        telemetry.addData(" driverRightStick tgt pwr", " driverRightStick pwr: " + String.format("%.2f", rightY));
    }



        }
