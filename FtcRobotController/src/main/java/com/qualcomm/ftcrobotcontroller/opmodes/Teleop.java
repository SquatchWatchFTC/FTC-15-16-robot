/* Copyright (c) 2014 Qualcomm Technologies Inc

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Qualcomm Technologies Inc nor the names of its contributors
may be used to endorse or promote products derived from this software without
specific prior written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. */

package com.qualcomm.ftcrobotcontroller.opmodes;

import android.hardware.SensorManager;

import com.qualcomm.hardware.HiTechnicNxtUltrasonicSensor;
import com.qualcomm.hardware.ModernRoboticsDigitalTouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;
import com.qualcomm.robotcore.util.Range;

import java.nio.channels.Channel;

/**
 * TeleOp Mode
 * <p>
 * Enables control of the robot via the gamepad
 */

public class Teleop extends OpMode {



	final static double BOX_MIN_RANGE = 0.0;
	final static double BOX_MAX_RANGE = 1.0;

	final static double BRIGHT_MIN_RANGE = 0.0;
	final static double BRIGHT_MAX_RANGE = 1.0;

	final static double BLEFT_MIN_RANGE = 0.0;
	final static double BLEFT_MAX_RANGE = 1.0;

	final static double MENLEFT_MIN_RANGE = 0.0;
	final static double MENLEFT_MAX_RANGE = 1.0;

	final static double MENRIGHT_MIN_RANGE = 0.0;
	final static double MENRIGHT_MAX_RANGE = 1.0;

	final static double DUMP_MIN_RANGE = 0.10;
	final static double DUMP_MAX_RANGE = 1.0;

	final static double LEFT_MIN_RANGE = 0.0;
	final static double LEFT_MAX_RANGE = 1.0;

	final static double RED_MIN_RANGE = 0.0;
	final static double RED_MAX_RANGE = 1.0;


	double boxPosition;

	double boxDelta = 0.003;

	double brightPosition;
	double brightDelta = 0.2;

	double bleftPosition;
	double bleftDelta = 0.2;



	double menleftPosition;
	double menleftDelta = 0.005;

	double menrightPosition;
	double menrightDelta = 0.005;

	double dumpPosition;
	//double dumpPosition2;
	double dumpDelta = 1.0;
	double dumpdelta2 = 0.25;

	double leftPosition;
	double rightPosition;



	//Drive Train
	DcMotor leftMotor;
	DcMotor rightMotor;

	DcMotor motor_1;
	DcMotor motor_2;


	Servo menleft;
	Servo box;
	Servo menright;
	Servo dump;
	Servo bright;
	Servo bleft;




	ColorSensor colorSensor;
	GyroSensor GyroSensor;


	//UltrasonicSensor Distance;

	public Teleop() {
	}


	@Override
	public void init() {

		rightMotor = hardwareMap.dcMotor.get("right");
		leftMotor = hardwareMap.dcMotor.get("left");
		rightMotor.setDirection(DcMotor.Direction.REVERSE);//CHANGED RIGHT


		//renaming slide

		motor_1 = hardwareMap.dcMotor.get("leftslide");
		motor_2 = hardwareMap.dcMotor.get("rightslide");
		motor_2.setDirection(DcMotor.Direction.REVERSE);



		box = hardwareMap.servo.get("box");
		boxPosition = 0.5;

		bright = hardwareMap.servo.get("bright");
		brightPosition = 0.1;

		bleft = hardwareMap.servo.get("bleft");
		bleftPosition=0.0;

		menleft = hardwareMap.servo.get("flipperleft");
		menleftPosition = 1;

		menright = hardwareMap.servo.get("flipperright");
		menrightPosition = 0.0;

		dump = hardwareMap.servo.get("mendump");
		dumpPosition = 0.10;
		//dumpPosition2= 0.10;

		GyroSensor = hardwareMap.gyroSensor.get("gyro");
		GyroSensor.calibrate();
		GyroSensor.resetZAxisIntegrator();





		colorSensor= hardwareMap.colorSensor.get("Color");
		//Distance= hardwareMap.ultrasonicSensor.get("Distance");
















	}

	@Override
	public void loop() {


		float leftY = gamepad1.right_stick_y;
		float rightY = gamepad1.left_stick_y;

		if (gamepad1.left_trigger == 1) {

			leftMotor.setPower(leftY*0.55);
			rightMotor.setPower(rightY*0.55);
		}
		else if (gamepad1.right_trigger == 1)
		{
			leftMotor.setPower(leftY*0.45);
			rightMotor.setPower(rightY*0.45);

		}

		else {
			leftMotor.setPower(leftY);
			rightMotor.setPower(rightY);
		}
		// write the values to the motors



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
		else if (gamepad2.x) {
			boxPosition -= boxDelta;

		}
		boxPosition = Range.clip(boxPosition, BOX_MIN_RANGE, BOX_MAX_RANGE);

		// write position values to the wrist and claw servo
		box.setPosition(boxPosition);


		if (gamepad1.b) {
			menleftPosition += menleftDelta;

		}
		else if (gamepad1.a) {
			menleftPosition -= menleftDelta;

		}
		menleftPosition = Range.clip(menleftPosition, MENLEFT_MIN_RANGE, MENLEFT_MAX_RANGE);

		// write position values to the wrist and claw servo
		menleft.setPosition(menleftPosition);


		if (gamepad1.x) {
			menrightPosition += menrightDelta;

		}
		else if (gamepad1.y) {
			menrightPosition -= menrightDelta;

		}
		menrightPosition = Range.clip(menrightPosition, MENRIGHT_MIN_RANGE, MENRIGHT_MAX_RANGE);

		// write position values to the wrist and claw servo
		menright.setPosition(menrightPosition);


		if (gamepad2.left_bumper) {
			dumpPosition += dumpDelta;

		}
		else if (gamepad2.right_bumper) {
			dumpPosition -= dumpDelta;

		}
		dumpPosition = Range.clip(dumpPosition, DUMP_MIN_RANGE, DUMP_MAX_RANGE);

		// write position values to the wrist and claw servo
		dump.setPosition(dumpPosition);

		if (gamepad2.left_trigger ==1) {
			bleftPosition -= bleftDelta;

		}
		bleftPosition = Range.clip(bleftPosition, BLEFT_MIN_RANGE, BLEFT_MAX_RANGE);

		// write position values to the wrist and claw servo
		bleft.setPosition(bleftPosition);

		if (gamepad2.right_trigger==1) {
			brightPosition -= brightDelta;

		}
		brightPosition = Range.clip(brightPosition, BRIGHT_MIN_RANGE, BRIGHT_MAX_RANGE);

		// write position values to the wrist and claw servo
		bright.setPosition(brightPosition);



		/*
		 * Send telemetry data back to driver station. Note that if we are using
		 * a legacy NXT-compatible motor controller, then the getPower() method
		 * will return a null value. The legacy NXT-compatible motor controllers
		 * are currently write only.
		 */
		telemetry.addData("Text", "*** Robot Data***");
		telemetry.addData("left tgt pwr", "Left  pwr: " + String.format("%.2f", leftY));
		telemetry.addData(" driverRightStick tgt pwr", " driverRightStick pwr: " + String.format("%.2f", rightY));
		telemetry.addData("Red  ", colorSensor.red());
		telemetry.addData("Blue ", colorSensor.blue());
		telemetry.addData("Heading", GyroSensor.getHeading());
		telemetry.addData("x", GyroSensor.rawX());
		telemetry.addData("y", GyroSensor.rawY());
		telemetry.addData("z", GyroSensor.rawZ());

		//telemetry.addData("Distance: ", Distance.getUltrasonicLevel());




	}
}

	/*
	 * Code to run when the op mode is first disabled goes here
	 *
	 * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#stop()
	 */






