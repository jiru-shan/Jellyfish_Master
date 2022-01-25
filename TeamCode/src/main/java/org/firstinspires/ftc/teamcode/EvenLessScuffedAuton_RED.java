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
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDriveCancelable;

@Autonomous
@Config
public class EvenLessScuffedAuton_RED extends LinearOpMode
{
    DcMotorEx test;
    ColorRangeSensor sensorRange1, sensorRange2, driveLeft, driveRight;

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

    double CYCLE_TIME=4;

    enum IntakeState {GETTING,
        RETURNING,
    }

    enum LiftState
    {
        LING,
        RETURNING,
        DONE
    }

    int alliance_targetTipped = 700;

    double i_hate_existence;

    SampleMecanumDriveCancelable drive;

    Trajectory goingTrajectory;
    Trajectory returningTrajectory;
    IntakeState state;
    //LiftState lstate;

    @Override
    public void runOpMode() throws InterruptedException {
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
        driveLeft=hardwareMap.get(ColorRangeSensor.class, "driveSensor1");
        driveRight=hardwareMap.get(ColorRangeSensor.class, "driveSensor2");

        drive = new SampleMecanumDriveCancelable(hardwareMap);
        Pose2d startPose = new Pose2d(0, 0, 0);
        ElapsedTime time = new ElapsedTime();
        drive.setPoseEstimate(startPose);


        waitForStart();

        lowerIntakes();
        Trajectory prepreTrajectory=drive.trajectoryBuilder(startPose)
                .forward(7)
                .build();
        drive.followTrajectory(prepreTrajectory);
        drive.setPoseEstimate(new Pose2d(0,0,Math.toRadians(0)));


        while(30-time.seconds()>CYCLE_TIME)
        {
        double pathChange=0;
        Trajectory preTrajectory = drive.trajectoryBuilder(startPose)
                .lineToSplineHeading(new Pose2d(-49, -1,Math.toRadians(5)))
                .build();
        drive.followTrajectory(preTrajectory);
        state=IntakeState.GETTING;
        goingTrajectory=drive.trajectoryBuilder(preTrajectory.end())
                .lineToSplineHeading(new Pose2d(-71, -3.25, Math.toRadians(10)), SampleMecanumDrive.getVelocityConstraint(40,
                        DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        drive.followTrajectoryAsync(goingTrajectory);
        pathChange=1;
        while(!hasBlock()&&pathChange<3)
        {
            switch(state)
            {
                case GETTING:
                    if(!drive.isBusy())
                    {

                        state=IntakeState.RETURNING;
                        returningTrajectory= drive.trajectoryBuilder(goingTrajectory.end())
                                .lineToSplineHeading(new Pose2d(-49, -1, Math.toRadians(5)))
                                .build();
                        drive.followTrajectoryAsync(returningTrajectory);
                    }
                    break;
                case RETURNING:
                    if(!drive.isBusy())
                    {
                        state=IntakeState.GETTING;
                        goingTrajectory=drive.trajectoryBuilder(returningTrajectory.end())
                                .lineToSplineHeading(new Pose2d(-71-2*pathChange, -3.25-pathChange, Math.toRadians(10+(2.5*pathChange))), SampleMecanumDrive.getVelocityConstraint(40,
                                        DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                                .build();
                        drive.followTrajectory(goingTrajectory);
                        pathChange++;
                    }
                    break;
            }
            drive.update();
            rightIntake.setPower(1);
        }
        double target=SystemClock.uptimeMillis()+300;
        while(SystemClock.uptimeMillis()<target)
        {
            rightIntake.setPower(1);
            drive.update();
        }
        drive.cancelFollowing();
        rightIntake.setPower(0);
        raiseIntakes();

        returningTrajectory=drive.trajectoryBuilder(drive.getPoseEstimate())
                .lineToSplineHeading(new Pose2d(-42, -1, Math.toRadians(0)))
                .build();
        drive.followTrajectory(returningTrajectory);

        Trajectory maldTrajectory=drive.trajectoryBuilder(returningTrajectory.end())
                .strafeLeft(10, SampleMecanumDrive.getVelocityConstraint(40,
                        DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
        drive.followTrajectory(maldTrajectory);

        drive.setPoseEstimate(new Pose2d(drive.getPoseEstimate().getX(), 0, 0));

        //lstate=LiftState.LING;
        Trajectory lineTrajectory=drive.trajectoryBuilder(drive.getPoseEstimate())
                .lineToSplineHeading(startPose)
                .build();
        drive.followTrajectory(lineTrajectory);
        /*while(lstate!=LiftState.DONE)
        {

            switch(lstate)
            {
                case LING:
                    if(onColor()||!drive.isBusy())
                    {
                        telemetry.addData(">", "?");
                        telemetry.update();
                        lstate=LiftState.RETURNING;
                        drive.cancelFollowing();
                        drive.setPoseEstimate(new Pose2d(-46, 0, 0));
                        Trajectory returnTrajectory=drive.trajectoryBuilder(drive.getPoseEstimate())
                                .lineTo(new Vector2d(0,0))
                                .build();
                        drive.followTrajectoryAsync(returnTrajectory);
                    }
                    break;
                case RETURNING:
                    if(!drive.isBusy())
                    {
                        //lift();
                        lstate=LiftState.DONE;
                    }
                    break;
                case DONE:
                    break;
            }
*/
        }
    }
    public void lowerIntakes()
    {
        i_topRight.setPosition(i_minRange_topRight);
        i_bottomRight.setPosition(i_minRange_bottomRight);
    }
    public void raiseIntakes()
    {
        i_topRight.setPosition(i_maxRange_topRight);
        i_bottomRight.setPosition(i_maxRange_bottomRight);
    }
    public boolean onColor()
    {
        if(rgbAvg(driveLeft)>215||rgbAvg(driveRight)>215)
        {
            return true;
        }
        return false;
    }
    public double rgbAvg(ColorRangeSensor pain)
    {
        return (pain.green()+pain.blue()+pain.red())/3;
    }

    public boolean hasBlock()
    {

        if (sensorRange1.getDistance(DistanceUnit.MM) < 55 || sensorRange2.getDistance(DistanceUnit.MM) < 55) {
            return true;
        }
        return false;
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
