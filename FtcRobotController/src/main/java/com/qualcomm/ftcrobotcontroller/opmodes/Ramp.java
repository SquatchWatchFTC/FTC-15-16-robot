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
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.IrSeekerSensor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * A simple example of a linear op mode that will approach an IR beacon
 */
public class Ramp extends OpMode {


final static double ENCODERS_CPR= 1440;

    boolean firstTime = true;
    boolean lastTime = false;
    int state = 1;

    double boxPosition;

    double boxDelta = 0.003;

    double menleftPosition;
    double menleftDelta = 0.005;

    double menrightPosition;
    double menrightDelta = 0.005;

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
    public void stop(){
        leftMotor.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        rightMotor.setMode(DcMotorController.RunMode.RESET_ENCODERS);
    }

  public void init(){
      motor_1 = hardwareMap.dcMotor.get("leftslide");
      motor_2 = hardwareMap.dcMotor.get("rightslide");
      motor_2.setDirection(DcMotor.Direction.REVERSE);


      box = hardwareMap.servo.get("box");
      boxPosition = 1.2;

      menleft = hardwareMap.servo.get("flipperleft");
      menleftPosition = 1;

      menright = hardwareMap.servo.get("flipperright");
      menrightPosition = 0.0;

      dump = hardwareMap.servo.get("mendump");
      dumpPosition = 0.15;rightMotor = hardwareMap.dcMotor.get("right");
      leftMotor = hardwareMap.dcMotor.get("left");
      rightMotor.setDirection(DcMotor.Direction.REVERSE);//CHANGED RIGHT


      //renaming slide




      leftMotor.setMode(DcMotorController.RunMode.RESET_ENCODERS);
      rightMotor.setMode(DcMotorController.RunMode.RESET_ENCODERS);
  }
    public void driveStraight(double speed, int distance){
        if(firstTime) {


            leftMotor.setMode(DcMotorController.RunMode.RESET_ENCODERS);
            rightMotor.setMode(DcMotorController.RunMode.RESET_ENCODERS);
            leftMotor.setTargetPosition (Math.abs(distance));
            rightMotor.setTargetPosition (Math.abs(distance));

            leftMotor.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
            rightMotor.setMode(DcMotorController.RunMode.RUN_TO_POSITION);

            leftMotor.setPower(speed );
            rightMotor.setPower(speed);
            firstTime = false;
        }
        if(Math.abs(leftMotor.getCurrentPosition()) > leftMotor.getTargetPosition()) {
            leftMotor.setPower(0.0);
            rightMotor.setPower(0.0);
            lastTime = true;
        }
    }

    public void rightturn(double speed,  int distance) {
        if (firstTime) {
            leftMotor.setChannelMode(DcMotorController.RunMode.RESET_ENCODERS);
            rightMotor.setChannelMode(DcMotorController.RunMode.RESET_ENCODERS);
            leftMotor.setTargetPosition(distance);
            rightMotor.setTargetPosition(distance);


            leftMotor.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
            rightMotor.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);

            leftMotor.setPower(speed);
            rightMotor.setPower(speed * -1);
            firstTime = false;
        }

        if (Math.abs(rightMotor.getCurrentPosition()) > rightMotor.getTargetPosition()) {
            leftMotor.setPower(0.0);
            rightMotor.setPower(0.0);
            lastTime = true;

        }

    }


    

    public void loop(){
        // wait for the start button to be pressed
        switch (state) {

            case 1:
                driveStraight(1, 5000);


                if (lastTime) {


                    telemetry.addData("stopped", "");
                    telemetry.addData("motor power", leftMotor.getPower());

                    leftMotor.setMode(DcMotorController.RunMode.RESET_ENCODERS);
                    rightMotor.setMode(DcMotorController.RunMode.RESET_ENCODERS);
                    if (rightMotor.getCurrentPosition() == 0) {
                        firstTime = true;
                        lastTime = false;
                        state++;
                    }

                }
                telemetry.clearData();
                break;




        case 2:
        rightturn(0.5, 5000);
        if (lastTime){

            telemetry.addData("stopped", "");
            telemetry.addData("motor power", leftMotor.getPower());

            leftMotor.setMode(DcMotorController.RunMode.RESET_ENCODERS);
            rightMotor.setMode(DcMotorController.RunMode.RESET_ENCODERS);
            if (rightMotor.getCurrentPosition() == 0) {
                firstTime = true;
                lastTime = false;
                state++;

        }}

        telemetry.clearData();
        break;



    }}







/*
      //Motor wait for 2 seconds before starting
      leftMotor.setPower(0);
      rightMotor.setPower(0);
      sleep(2000);*/

        //Encoders running to forward position
        //leftMotor.setChannelMode(DcMotorController.RunMode.RUN_TO_POSITION);
        //rightMotor.setChannelMode(DcMotorController.RunMode.RUN_TO_POSITION);

        //Power for forward position
        //leftMotor.setPower(1.0);
        //rightMotor.setPower(1.0);




        //Motors stop for transition 2 seconds
        //leftMotor.setPower(0);
        //rightMotor.setPower(0);
        //sleep(2000);



    }