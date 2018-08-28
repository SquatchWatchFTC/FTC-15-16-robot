/* Copyright (c) 2015 Qualcomm Technologies Inc

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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.Servo;


/**
 * A simple example of a linear op mode that will approach an IR beacon
 */
public class BlueGyro extends LinearOpMode {



    double boxPosition;

    double boxDelta = 0.003;

    double menleftPosition;
    double menleftDelta = 0.005;

    double menrightPosition;
    double menrightDelta = 0.005;

    double dumpPosition;
    //double dumpPosition2;
    double dumpDelta = 1.0;
    double dumpdelta2 = 0.25;


    //Drive Train
    DcMotor leftMotor;
    DcMotor rightMotor;


    Servo dump;
    Servo menleft;
    Servo box;
    Servo menright;

    GyroSensor GyroSensor;



    final static double ENCODER_CPR = 1120;



@Override
public void runOpMode() throws InterruptedException {


    leftMotor = hardwareMap.dcMotor.get("left");
    rightMotor = hardwareMap.dcMotor.get("right");
    rightMotor.setDirection(DcMotor.Direction.REVERSE);
    dump = hardwareMap.servo.get("mendump");


    box = hardwareMap.servo.get("box");
    menleft = hardwareMap.servo.get("flipperleft");
    menright = hardwareMap.servo.get("flipperright");

    GyroSensor = hardwareMap.gyroSensor.get("gyro");


    leftMotor.setMode(DcMotorController.RunMode.RESET_ENCODERS);
    rightMotor.setMode(DcMotorController.RunMode.RESET_ENCODERS);
    GyroSensor.calibrate();

    waitForStart();
    if (GyroSensor.rawX() < 10) {
        leftMotor.setPower(0);
        rightMotor.setPower(0);
    } else {

        dump.setPosition(0.10);
        box.setPosition(0.5);
        menleft.setPosition(1);
        menright.setPosition(0.0);

        sleep(0100);

        leftMotor.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        rightMotor.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);

        leftMotor.setPower(0.40);
        rightMotor.setPower(0.40);

        while (Math.abs(leftMotor.getCurrentPosition()) < 8500) {
            telemetry.addData("Heading", GyroSensor.getHeading());
            telemetry.addData("Left Enc:", leftMotor.getCurrentPosition()); //drive forward
            telemetry.addData("Right Enc:", rightMotor.getCurrentPosition());
            waitOneFullHardwareCycle();
        }


        leftMotor.setPower(0.35);
        rightMotor.setPower(-0.35);

        while (GyroSensor.getHeading() < 43) {
            telemetry.addData("Gyro", GyroSensor.getHeading());
            telemetry.addData("Left Enc:", leftMotor.getCurrentPosition()); //turn right
            telemetry.addData("Right Enc:", rightMotor.getCurrentPosition());
            waitOneFullHardwareCycle();
        }

        leftMotor.setPower(0);
        rightMotor.setPower(0);
        sleep(0100);

        leftMotor.setPower(0.40);
        rightMotor.setPower(0.40);
        double a = leftMotor.getCurrentPosition();

        while (Math.abs(leftMotor.getCurrentPosition()) < a + 14500) {
            telemetry.addData("Heading", GyroSensor.getHeading());
            telemetry.addData("x", GyroSensor.rawX());
            telemetry.addData("y", GyroSensor.rawY());
            telemetry.addData("z", GyroSensor.rawZ());

            telemetry.addData("Left Enc:", leftMotor.getCurrentPosition()); //second drive forward
            telemetry.addData("Right Enc:", rightMotor.getCurrentPosition());
            waitOneFullHardwareCycle();
        }


        leftMotor.setPower(0.35);
        rightMotor.setPower(-0.35);


        while (GyroSensor.getHeading() < 86) {
            telemetry.addData("Gyro", GyroSensor.getHeading());
            telemetry.addData("x", GyroSensor.rawX());
            telemetry.addData("y", GyroSensor.rawY());
            telemetry.addData("z", GyroSensor.rawZ());
            telemetry.addData("Left Enc:", leftMotor.getCurrentPosition()); //second drive forward
            telemetry.addData("Right Enc:", rightMotor.getCurrentPosition());
            waitOneFullHardwareCycle();
        }

        leftMotor.setPower(0);
        rightMotor.setPower(0);
        sleep(0100);

        leftMotor.setPower(0.40);
        rightMotor.setPower(0.40);
        double b = leftMotor.getCurrentPosition();

        while (Math.abs(leftMotor.getCurrentPosition()) < b + 800) {
            telemetry.addData("Gyro", GyroSensor.getHeading());
            telemetry.addData("x", GyroSensor.rawX());
            telemetry.addData("y", GyroSensor.rawY());
            telemetry.addData("z", GyroSensor.rawZ());
            telemetry.addData("Left Enc:", leftMotor.getCurrentPosition()); //second drive forward
            telemetry.addData("Right Enc:", rightMotor.getCurrentPosition());
            waitOneFullHardwareCycle();
        }

        leftMotor.setPower(0);
        rightMotor.setPower(0);
        sleep(0500);


        dump.setPosition(1.0);
        sleep(800);

        dump.setPosition(0.10);
        sleep(800);
        waitOneFullHardwareCycle();


        leftMotor.setPower(0.35);
        rightMotor.setPower(-0.35);

        while (GyroSensor.getHeading() < 270) {
            telemetry.addData("Gyro", GyroSensor.getHeading());//turn around
            telemetry.addData("x", GyroSensor.rawX());
            telemetry.addData("y", GyroSensor.rawY());
            telemetry.addData("z", GyroSensor.rawZ());
            telemetry.addData("Left Enc:", leftMotor.getCurrentPosition());
            telemetry.addData("Right Enc:", rightMotor.getCurrentPosition());
            waitOneFullHardwareCycle();
        }


        leftMotor.setPower(0);
        rightMotor.setPower(0);
        sleep(1000);


    }
}



















}










