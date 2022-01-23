package org.firstinspires.ftc.teamcode;

import android.os.SystemClock;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous
@Config
public class EvenLessScuffedAuton_BLUE extends LinearOpMode
{
    public static double TURN_ANGLE=210;
    public static double CYCLE_TIME;
    DcMotorEx test;
    ColorRangeSensor sensorRange1, sensorRange2;

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

    double i_minRange_topRight = 0.96;
    double i_maxRange_topRight = 0.17;
    double i_minRange_bottomRight = 0.04;
    double i_maxRange_bottomRight = 0.83;

    double i_minRange_topLeft = 0.1;
    double i_maxRange_topLeft = 0.85;
    double i_minRange_bottomLeft = 0.9;
    double i_maxRange_bottomLeft = 0.15;

    int alliance_targetTipped = 700;

    double i_hate_existence;

    SampleMecanumDrive drive;

    //TODO: Fix all the values for this side

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

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
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

        sensorRange1 = hardwareMap.get(ColorRangeSensor.class, "colorSensor_right");
        sensorRange2 = hardwareMap.get(ColorRangeSensor.class, "colorSensor_left");

        drive=new SampleMecanumDrive(hardwareMap);
        Pose2d startPose=new Pose2d(0,0,0);
        ElapsedTime time=new ElapsedTime();
        drive.setPoseEstimate(startPose);

        waitForStart();

        //check which zone for vision

        i_topRight.setPosition(i_minRange_topRight);
        i_bottomRight.setPosition(i_minRange_bottomRight);

            Trajectory preStart=drive.trajectoryBuilder(new Pose2d())
                    .forward(7)
                    .build();
            drive.followTrajectory(preStart);
            drive.setPoseEstimate(new Pose2d(0,0,0));

            //place vision cube now


            while(30000-time.milliseconds()>CYCLE_TIME)
            {

                Trajectory primaryTrajectory0 = drive.trajectoryBuilder(new Pose2d())

                        .lineTo(new Vector2d(-52, 0))
                        .build();
                Trajectory primaryTrajectory1 = drive.trajectoryBuilder(primaryTrajectory0.end())
                        .strafeRight(5)
                        .splineTo(new Vector2d(-57, -20), Math.toRadians(TURN_ANGLE))
                        .build();
                Trajectory primaryTrajectory2 = drive.trajectoryBuilder(primaryTrajectory1.end())
                        .lineTo(new Vector2d(-75, -19))
                        .build();


                drive.followTrajectory(primaryTrajectory0);
                drive.followTrajectory(primaryTrajectory1);
                drive.followTrajectory(primaryTrajectory2);
                getCube();
                Trajectory returningTrajectory0 = drive.trajectoryBuilder((drive.getPoseEstimate()))
                        .lineToLinearHeading(primaryTrajectory2.end())
                        .build();
                drive.followTrajectory(returningTrajectory0);

                Trajectory returningTrajectory1 = drive.trajectoryBuilder(returningTrajectory0.end())
                        .splineTo(new Vector2d(-57, -10), Math.toRadians(0))
                        .build();
                drive.followTrajectory(returningTrajectory1);

                Trajectory returningTrajectory2 = drive.trajectoryBuilder(returningTrajectory1.end())
                        .strafeLeft(12)
                        .build();

                drive.followTrajectory(returningTrajectory2);

                drive.setPoseEstimate(new Pose2d(drive.getPoseEstimate().getX(), 0, 0));

                //fix localization by aligning with color sensor on white tape

                Trajectory returningTrajectory3 = drive.trajectoryBuilder(drive.getPoseEstimate())
                        .lineTo(new Vector2d(0, 0))
                        .build();
                drive.followTrajectory(returningTrajectory3);

                //deposit cubes
            }


            Trajectory park=drive.trajectoryBuilder(new Pose2d())
                    .lineTo(new Vector2d(-52, 0))
                    .build();
            drive.followTrajectory(park);

    }


    public boolean hasBlock() {

        if (sensorRange1.getDistance(DistanceUnit.MM) < 55 || sensorRange2.getDistance(DistanceUnit.MM) < 55) {
            return true;
        }
        return false;
    }

    public void getCube() {
        int start = leftFront.getCurrentPosition();
        int end = 0;



        drive.update();
        while(hasBlock() == false) {
            leftFront.setPower(-0.3);
            leftBack.setPower(-0.3);
            rightFront.setPower(-0.3);
            rightBack.setPower(-0.3);
            rightIntake.setPower(1);
            drive.update();
        }
        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);
        rightIntake.setPower(0);
        end = leftFront.getCurrentPosition();
        int distance = end - start;
        drive.update();
        i_hate_existence= -1*1.89 * 2 * Math.PI * 1 * distance / 384.5;
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
