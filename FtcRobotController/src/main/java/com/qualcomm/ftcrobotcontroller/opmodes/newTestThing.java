
package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;


/**
 * Created by 8868 on 8/24/2016.
 */
public class newTestThing extends LinearOpMode {


    final static double MENLEFT_MIN_RANGE = 0.0;
    final static double MENLEFT_MAX_RANGE = 1.0;

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

    double brightPosition;
    double brightDelta = 1.0;

    double bleftPosition;
    double bleftDelta = 1.0;

    //Drive Train
    DcMotor leftMotor;
    DcMotor rightMotor;


    Servo dump;
    Servo menleft;
    Servo box;
    Servo menright;
    Servo bright;
    Servo bleft;
    ColorSensor colorSensor;
    final static double ENCODER_CPR = 1120;


    @Override
    public void runOpMode() throws InterruptedException {


        leftMotor = hardwareMap.dcMotor.get("left");
        rightMotor = hardwareMap.dcMotor.get("right");
        rightMotor.setDirection(DcMotor.Direction.REVERSE);
        dump = hardwareMap.servo.get("mendump");


        box = hardwareMap.servo.get("box");
        menleft = hardwareMap.servo.get("flipperleft");
        menleftPosition = 1;
        menright = hardwareMap.servo.get("flipperright");
        menrightPosition = 0.0;

        bright = hardwareMap.servo.get("bright");
        bleft = hardwareMap.servo.get("bleft");

        colorSensor = hardwareMap.colorSensor.get("Color");


        leftMotor.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        rightMotor.setMode(DcMotorController.RunMode.RESET_ENCODERS);

        waitForStart();
        dump.setPosition(0.10);
        box.setPosition(0.5);
        menleft.setPosition(1);
        menright.setPosition(0.0);
        bright.setPosition(0.1);
        bleft.setPosition(0.0);

        sleep(0100);

        leftMotor.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        rightMotor.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);

        leftMotor.setPower(0.40);
        rightMotor.setPower(0.40);

        while (Math.abs(leftMotor.getCurrentPosition()) < 8500) {
            telemetry.addData("Left Enc:", leftMotor.getCurrentPosition()); //drive forward
            telemetry.addData("Right Enc:", rightMotor.getCurrentPosition());
            waitOneFullHardwareCycle();
        }


        leftMotor.setPower(0);
        rightMotor.setPower(0);
        double a = leftMotor.getCurrentPosition();


        if (colorSensor.red() > colorSensor.blue()) {

            leftMotor.setPower(-0.40);
            rightMotor.setPower(-0.40);
            double f = leftMotor.getCurrentPosition();

            while (leftMotor.getCurrentPosition() > f - 0050) {
                telemetry.addData("Left Enc:", leftMotor.getCurrentPosition()); //back up
                telemetry.addData("Right Enc:", rightMotor.getCurrentPosition());
                waitOneFullHardwareCycle();
            }


            leftMotor.setPower(0);
            rightMotor.setPower(0);
            sleep(0100);

            menleftPosition = Range.clip(menleftPosition, MENLEFT_MIN_RANGE, MENLEFT_MAX_RANGE);

            menleft.setPosition(.75);


        } else {


            leftMotor.setPower(-0.40);
            rightMotor.setPower(-0.40);
            double f = leftMotor.getCurrentPosition();

            while (leftMotor.getCurrentPosition() > f - 0050) {
                telemetry.addData("Left Enc:", leftMotor.getCurrentPosition()); //back up ramp
                telemetry.addData("Right Enc:", rightMotor.getCurrentPosition());
                waitOneFullHardwareCycle();
            }


            leftMotor.setPower(0);
            rightMotor.setPower(0);
            sleep(0100);

            menrightPosition = Range.clip(menleftPosition, MENLEFT_MIN_RANGE, MENLEFT_MAX_RANGE);

            menright.setPosition(.75);



            waitOneFullHardwareCycle();

        }
    }
}

