package org.firstinspires.ftc.teamcode.past_code;

import android.os.SystemClock;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.hardware.ServoControllerEx;

@Config
@TeleOp
public class ServoTesting extends LinearOpMode
{
    Servo d_open;
    Servo d_coverLeft;
    Servo d_coverRight;
    Servo d_bendLeft;
    Servo d_bendRight;
    Servo i_topLeft;
    Servo i_bottomLeft;
    Servo i_topRight;
    Servo i_bottomRight;

    DcMotor rightIntake;

    public static double d_open_minRange = 0.66;
    double d_open_top = 0.45;
    public static double d_minRange_bendLeft = 0.94;      // need to fix bend values
    double d_maxRange_bendLeft = 0.78;
    public static double d_minRange_bendRight = 0.06;
    double d_maxRange_bendRight = 0.21;
    double d_open_minRangeSemi = 0.63;

    double i_minRange_topRight = 0.96;
    public static double i_maxRange_topRight = 0.20;
    double i_minRange_bottomRight = 0.04;
    public static double i_maxRange_bottomRight = 0.80;

    double d_minRange_coverLeft = 0.55;
    double d_minRange_coverRight = 0.45;
    double d_maxRange_coverRight = 0.85;

    double i_minRange_topLeft = 0.1;
    double i_maxRange_topLeft = 0.85;
    double i_minRange_bottomLeft = 0.9;
    double i_maxRange_bottomLeft = 0.15;
    ServoController servoController;

    @Override
    public void runOpMode() throws InterruptedException
    {
        d_open = hardwareMap.servo.get("d_open");
        d_coverLeft = hardwareMap.servo.get("d_coverLeft");
        d_coverRight = hardwareMap.servo.get("d_coverRight");
        d_bendLeft = hardwareMap.servo.get("d_bendLeft");
        d_bendRight = hardwareMap.servo.get("d_bendRight");
        i_topLeft = hardwareMap.servo.get("i_topLeft");
        i_bottomLeft = hardwareMap.servo.get("i_bottomLeft");
        i_topRight = hardwareMap.servo.get("i_topRight");
        i_bottomRight = hardwareMap.servo.get("i_bottomRight");
        servoController=d_open.getController();
        int port=d_open.getPortNumber();

        rightIntake = hardwareMap.dcMotor.get("rightIntake");

        i_bottomLeft.setPosition(i_minRange_bottomLeft);
        i_topLeft.setPosition(i_minRange_topLeft);

        d_open.resetDeviceConfigurationForOpMode();
        waitForStart();

    }
}
