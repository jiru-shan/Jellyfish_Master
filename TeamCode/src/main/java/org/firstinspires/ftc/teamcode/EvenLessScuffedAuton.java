package org.firstinspires.ftc.teamcode;

import android.os.SystemClock;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;


public class EvenLessScuffedAuton extends LinearOpMode
{
    DcMotorEx test;
    RevColorSensorV3 sensorRange1, sensorRange2;

    DcMotorEx leftFront, leftBack, rightBack, rightFront;
    DcMotor lift_front, lift_back, leftIntake, rightIntake;
    CRServo carousel;
    Servo d_open;
    Servo d_coverLeft;
    Servo d_coverRight;
    Servo d_bendLeft;
    Servo d_bendRight;
    Servo i_topLeft;
    Servo i_bottomLeft;
    Servo i_topRight;
    Servo i_bottomRight;

    // Deposit servo positions
    double d_open_minRange = 0.65;
    double d_open_top = 0.53;
    double d_minRange_bendLeft = 0.89;      // need to fix bend values
    double d_maxRange_bendLeft = 0.78;
    double d_minRange_bendRight = 0.10;
    double d_maxRange_bendRight = 0.21;

    int alliance_targetTipped = 700;

    @Override
    public void runOpMode() throws InterruptedException
    {
        // Motors
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");
        lift_front = hardwareMap.dcMotor.get("lift_front");
        lift_back = hardwareMap.dcMotor.get("lift_back");
        leftIntake = hardwareMap.dcMotor.get("leftIntake");
        rightIntake = hardwareMap.dcMotor.get("rightIntake");

        // Servos
        d_open = hardwareMap.servo.get("d_open");
        d_coverLeft = hardwareMap.servo.get("d_coverLeft");
        d_coverRight = hardwareMap.servo.get("d_coverRight");
        d_bendLeft = hardwareMap.servo.get("d_bendLeft");
        d_bendRight = hardwareMap.servo.get("d_bendRight");
        i_topLeft = hardwareMap.servo.get("i_topLeft");
        i_bottomLeft = hardwareMap.servo.get("i_bottomLeft");
        i_topRight = hardwareMap.servo.get("i_topRight");
        i_bottomRight = hardwareMap.servo.get("i_bottomRight");
        carousel = hardwareMap.crservo.get("carousel");

        test.getCurrent(CurrentUnit.AMPS);
        sensorRange1 = hardwareMap.get(RevColorSensorV3.class, "colorSensor_right");
        sensorRange2 = hardwareMap.get(RevColorSensorV3.class, "colorSensor_left");
        SampleMecanumDrive drive=new SampleMecanumDrive(hardwareMap);
        Pose2d startPose=new Pose2d(0,0,0);
        ElapsedTime time=new ElapsedTime();
        drive.setPoseEstimate(startPose);
        //TrajectorySequence traqSequence=drive.trajectorySequenceBuilder(startPose).

    }

    public boolean hasBlock() {

        if (sensorRange1.getDistance(DistanceUnit.MM) < 10 || sensorRange2.getDistance(DistanceUnit.MM) < 10) {
            return true;
        }
        return false;
    }

    public int distance() {
        int start = leftFront.getCurrentPosition();
        int end = 0;

        while(hasBlock() == false) {
            leftFront.setPower(0.7);
            leftBack.setPower(0.7);
            rightFront.setPower(0.7);
            rightBack.setPower(0.7);
        }
        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);
        end = leftFront.getCurrentPosition();
        int distance = end - start;
        return distance;
    }

    public void lift(){
        lift_front.setTargetPosition(-alliance_targetTipped);
        lift_back.setTargetPosition(-alliance_targetTipped);
        lift_front.setMode(DcMotor.RunMode.RUN_TO_POSITION);   // Move to deposit position
        lift_back.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lift_front.setPower(-1.0);
        lift_back.setPower(-1.0);

        //Thread.sleep(1000);
        double target= SystemClock.uptimeMillis()+1000;
        while(SystemClock.uptimeMillis()<target)
        {
            //stall
        }

        // Lift up deposit
        d_bendLeft.setPosition(d_maxRange_bendLeft);
        d_bendRight.setPosition(d_maxRange_bendRight);

        // Open to deposit in top level of alliance hub
        d_open.setPosition(d_open_top);

        //Thread.sleep(500);
        target=SystemClock.uptimeMillis()+500;
        while(SystemClock.uptimeMillis()<target)
        {
            //stall
        }

        // Close & bend down deposit
        d_open.setPosition(d_open_minRange);
        d_bendLeft.setPosition(d_minRange_bendLeft);
        d_bendRight.setPosition(d_minRange_bendRight);

        //Thread.sleep(1000);
        target=SystemClock.uptimeMillis()+1000;
        while(SystemClock.uptimeMillis()<target)
        {
            //stall
        }

        // Retract arm to original position
        lift_front.setTargetPosition(0);
        lift_back.setTargetPosition(0);
        lift_front.setMode(DcMotor.RunMode.RUN_TO_POSITION);   // Move to deposit position
        lift_back.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lift_front.setPower(1.0);
        lift_back.setPower(1.0);

       // Thread.sleep(1000);
        target=SystemClock.uptimeMillis()+1000;
        while(SystemClock.uptimeMillis()<target)
        {
            //stall
        }


        lift_front.setPower(0);
        lift_back.setPower(0);
    }
}
