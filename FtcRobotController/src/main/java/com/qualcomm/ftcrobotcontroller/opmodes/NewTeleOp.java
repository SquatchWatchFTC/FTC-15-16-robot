package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

public class NewTeleOp extends OpMode {
    final static double BOX_MIN_RANGE = 0.0;
    final static double BOX_MAX_RANGE = 1.0;

    final static double MENLEFT_MIN_RANGE = 0.0;
    final static double MENLEFT_MAX_RANGE = 1.0;

    final static double MENRIGHT_MIN_RANGE = 0.0;
    final static double MENRIGHT_MAX_RANGE = 1.0;

    final static double DUMP_MIN_RANGE = 0.0;
    final static double DUMP_MAX_RANGE = 1.0;


    double boxPosition;

    double boxDelta = 0.01;

    double menleftPosition;
    double menleftDelta = 0.001;

    double menrightPosition;
    double menrightDelta = 0.001;

    double dumpPosition;
    double dumpDelta = 1.0;


    //Drive Train
    DcMotor leftMotor;
    DcMotor rightMotor;

    DcMotor motor_1;
    DcMotor motor_2;


    Servo menleft;
    Servo box;
    Servo menright;
    Servo dump;


    public NewTeleOp() {
    }


    @Override
    public void init() {

        rightMotor = hardwareMap.dcMotor.get("right");
        leftMotor = hardwareMap.dcMotor.get("left");
        rightMotor.setDirection(DcMotor.Direction.REVERSE);//CHANGED RIGHT


        //renaming slide

        motor_1 = hardwareMap.dcMotor.get("leftslide");
        motor_2 = hardwareMap.dcMotor.get("rightslide");


        box = hardwareMap.servo.get("box");
        boxPosition = 0.2;

        menleft = hardwareMap.servo.get("flipperleft");
        menleftPosition = 0.0;

        menright = hardwareMap.servo.get("flipperright");
        menrightPosition = 0.0;

        dump = hardwareMap.servo.get("mendump");
        dumpPosition = 0.0;

        hardwareMap.deviceInterfaceModule.get("distance");
    }

    @Override
    public void loop() {
        float leftY = -gamepad1.left_stick_y;
        float rightY = -gamepad1.right_stick_y;

        if (gamepad1.left_trigger == 1) {

            leftMotor.setPower(0.5);
            rightMotor.setPower(0.5);
        }
        // write the values to the motors

        else {
            leftMotor.setPower(leftY);
            rightMotor.setPower(rightY);
        }

        if (gamepad2.a) {
            motor_1.setPower(-1);
            motor_2.setPower(1);

        } else if (gamepad2.b) {
            motor_1.setPower(1);
            motor_2.setPower(-1);
        } else {
            motor_1.setPower(0);
            motor_2.setPower(0);

        }

        if (gamepad2.y) {
            boxPosition += boxDelta;

        }
        if (gamepad2.x) {
            boxPosition -= boxDelta;

        }
        boxPosition = Range.clip(boxPosition, BOX_MIN_RANGE, BOX_MAX_RANGE);

        // write position values to the wrist and claw servo
        box.setPosition(boxPosition);


        if (gamepad1.b) {
            menleftPosition += menleftDelta;

        }
        if (gamepad1.a) {
            menleftPosition -= menleftDelta;

        }
        menleftPosition = Range.clip(menleftPosition, MENLEFT_MIN_RANGE, MENLEFT_MAX_RANGE);

        // write position values to the wrist and claw servo
        menleft.setPosition(menleftPosition);


        if (gamepad1.x) {
            menrightPosition += menrightDelta;

        }
        if (gamepad1.y) {
            menrightPosition -= menrightDelta;

        }
        menrightPosition = Range.clip(menrightPosition, MENRIGHT_MIN_RANGE, MENRIGHT_MAX_RANGE);

        // write position values to the wrist and claw servo
        menright.setPosition(menrightPosition);


        if (gamepad2.left_bumper) {
            dumpPosition += dumpDelta;

        }
        if (gamepad2.right_bumper) {
            dumpPosition -= dumpDelta;

        }
        dumpPosition = Range.clip(dumpPosition, DUMP_MIN_RANGE, DUMP_MAX_RANGE);

        // write position values to the wrist and claw servo
        dump.setPosition(dumpPosition);



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

	/*
	 * Code to run when the op mode is first disabled goes here
	 *
	 * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#stop()
	 */
