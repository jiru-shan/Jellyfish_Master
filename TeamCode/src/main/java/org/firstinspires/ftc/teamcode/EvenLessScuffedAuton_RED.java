package org.firstinspires.ftc.teamcode;

import android.os.SystemClock;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.path.Path;
import com.acmerobotics.roadrunner.path.PathSegment;
import com.acmerobotics.roadrunner.path.QuinticSpline;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryGenerator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDriveCancelable;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.vision.VisionPipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@TeleOp
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

    //APPROACH 1: Start the deposit low so weighted cubes can make it in and then
    //increase the servo values so that deposit climbs higher for ball

    //APPROACH 2: Wide deposit and then clamp as soon as it stalls


    // Deposit servo positions
    double d_open_minRange = 0.52;
    double d_open_top = 0.4;
    double d_open_clamp=0.66;
    double d_minRange_bendLeft = 0.98;      // need to fix bend values
    double d_maxRange_bendLeft = 0.78;
    double d_minRange_bendRight = 0.02;
    double d_maxRange_bendRight = 0.21;

    double i_minRange_topRight = 0.96;
    double i_maxRange_topRight = 0.20;
    double i_minRange_bottomRight = 0.04;
    double i_maxRange_bottomRight = 0.80;

    double d_minRange_coverLeft = 0.55;
    double d_minRange_coverRight = 0.45;
    double d_maxRange_coverRight = 0.85;

    double prevTimer;
    double intakePower=1;

    OpenCvWebcam camera;
    VisionPipeline pipeline;

    double past_lift_value;
    double CYCLE_TIME=4;

    ElapsedTime time = new ElapsedTime();
    ElapsedTime intakeTimer=new ElapsedTime();

    enum GrabbingState {GETTING,
        RETURNING,
    }

    enum ReturningState
    {
        LING, //Ling arknights :)
        RETURNING,
        BALL,
        BALL_2,
        BALL_END,
        DONE
    }
    enum IntakeState
    {
        INTO_DEPOSIT, STALLING, EXTENDING_LIFT, BALL, BALL_2, DONE
    }

    int alliance_targetTipped = 1000;

    SampleMecanumDriveCancelable drive;

    int cubePos;

    Trajectory goingTrajectory;
    Trajectory returningTrajectory;
    GrabbingState GState;
    ReturningState RState;
    IntakeState IState;

    double intakeTarget=0;

    double intakeEjectDistance=30.0;

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
        driveLeft = hardwareMap.get(ColorRangeSensor.class, "driveSensor1");
        driveRight = hardwareMap.get(ColorRangeSensor.class, "driveSensor2");

        drive = new SampleMecanumDriveCancelable(hardwareMap);
        Pose2d startPose = new Pose2d(0, 0, 0);
        ElapsedTime time = new ElapsedTime();
        drive.setPoseEstimate(startPose);

        webcamInit();

        d_open.setPosition(d_open_minRange);
        d_bendLeft.setPosition(d_minRange_bendLeft);
        d_bendRight.setPosition(d_minRange_bendRight);

        d_coverLeft.setPosition(d_minRange_coverLeft);
        d_coverRight.setPosition(d_minRange_coverRight);

        i_bottomRight.setDirection(Servo.Direction.FORWARD);
        i_topRight.setDirection(Servo.Direction.FORWARD);

        waitForStart();

        cubePos= pipeline.getAnalysis();

        Trajectory prepreTrajectory = drive.trajectoryBuilder(startPose)
                .forward(7)
                .build();
        drive.followTrajectory(prepreTrajectory);
        drive.setPoseEstimate(new Pose2d(0, 0, Math.toRadians(0)));

        //Run roadrunner code to place in correct level and then return to starting pos
        //visionDeposit(cubePos);

        while (opModeIsActive())
        {

            drive.setPoseEstimate(new Pose2d(0, 0, Math.toRadians(0)));
            lowerIntakes();
            d_bendLeft.setPosition(d_minRange_bendLeft);
            d_bendRight.setPosition(d_minRange_bendRight);
            d_open.setPosition(d_open_minRange);
            double pathChange = 0;

            GState = GrabbingState.GETTING;
            goingTrajectory = drive.trajectoryBuilder(startPose, true)
                    .splineTo(new Vector2d(-49, -1), Math.toRadians(185))
                    .splineTo(new Vector2d(-73, -2.25), Math.toRadians(190), SampleMecanumDrive.getVelocityConstraint(40,
                            DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                    .build();

            drive.followTrajectoryAsync(goingTrajectory);
            pathChange = 1;
            while (!hasBlock() && pathChange < 3)
            {
                switch (GState) {
                    case GETTING:
                        if (!drive.isBusy()) {

                            GState = GrabbingState.RETURNING;
                            returningTrajectory = drive.trajectoryBuilder(goingTrajectory.end())
                                    .lineToSplineHeading(new Pose2d(-49, -1, Math.toRadians(5)))
                                    .build();
                            drive.followTrajectoryAsync(returningTrajectory);
                        }
                        break;
                    case RETURNING:
                        if (!drive.isBusy()) {
                            GState = GrabbingState.GETTING;
                            goingTrajectory = drive.trajectoryBuilder(returningTrajectory.end())
                                    .lineToSplineHeading(new Pose2d(-73 - 2 * pathChange, -2.25 - pathChange, Math.toRadians(10 + (2.5 * pathChange))), SampleMecanumDrive.getVelocityConstraint(40,
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
            double target = SystemClock.uptimeMillis() + 300;
            while (SystemClock.uptimeMillis() < target)
            {
                rightIntake.setPower(1);
                drive.update();
                raiseIntakes();
            }
            drive.cancelFollowing();
            rightIntake.setPower(0);


            returningTrajectory = drive.trajectoryBuilder(drive.getPoseEstimate())
                    .lineToSplineHeading(new Pose2d(-49, 8, Math.toRadians(0)))
                    .build();
            drive.followTrajectory(returningTrajectory);

            drive.setPoseEstimate(new Pose2d(drive.getPoseEstimate().getX(), 0, 0));

            RState = ReturningState.LING;
            IState=IntakeState.INTO_DEPOSIT;
            //intakeTarget=SystemClock.uptimeMillis()+2500;
            intakeTimer.reset();
            prevTimer=100;
            Trajectory lineTrajectory = drive.trajectoryBuilder(drive.getPoseEstimate())
                    .lineToSplineHeading(startPose, SampleMecanumDrive.getVelocityConstraint(25,
                            DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                    .build();
            drive.followTrajectoryAsync(lineTrajectory);
            while (RState != ReturningState.DONE)
            {

                switch (RState) {
                    case LING:
                        if (onColor())
                        {
                            RState = ReturningState.RETURNING;
                            drive.cancelFollowing();
                            drive.setPoseEstimate(new Pose2d(-38, 0, 0));
                            Trajectory returnTrajectory = drive.trajectoryBuilder(drive.getPoseEstimate())
                                    .lineTo(new Vector2d(0, 0))
                                    .build();
                            drive.followTrajectoryAsync(returnTrajectory);
                        }
                        else if(!drive.isBusy())
                        {
                            RState = ReturningState.RETURNING;
                        }
                        break;
                    case RETURNING:
                        if (!drive.isBusy()&&IState==EvenLessScuffedAuton_RED.IntakeState.DONE)
                        {
                            deposit();
                            RState = ReturningState.DONE;
                        }
                        break;

                    case DONE:
                        break;
                }
                drive.update();
                putInDeposit();
            }
       }
            /*Trajectory park=drive.trajectoryBuilder(drive.getPoseEstimate())
                .lineToSplineHeading(new Pose2d(-50, 0, Math.toRadians(0)))
                .build();
            drive.followTrajectory(park);*/

    }
    public boolean checkLift()
    {
        double change=727727727;
        if(time.milliseconds()>500)
        {
            change=Math.abs(Math.abs(lift_front.getCurrentPosition())-Math.abs(past_lift_value));
            past_lift_value=lift_front.getCurrentPosition();
            time.reset();
        }
        if(change<7.5)
        {
            return true;
        }
        return false;
    }
    public void lowerIntakes()
    {
        i_topRight.setPosition(i_minRange_topRight);
        i_bottomRight.setPosition(i_minRange_bottomRight);
    }

    public void putInDeposit()
    {
        if(IState!= EvenLessScuffedAuton_RED.IntakeState.DONE) {
            switch (IState) {
                case INTO_DEPOSIT:
                        d_open.setPosition(d_open_minRange);
                        d_bendLeft.setPosition(d_minRange_bendLeft);
                        d_bendRight.setPosition(d_minRange_bendRight);
                        d_coverRight.setPosition(d_maxRange_coverRight);
                        d_coverLeft.setPosition(d_minRange_coverLeft);
                        rightIntake.setPower(-intakePower);
                    if (intakeTimer.milliseconds()>2000)
                    {
                        IState = EvenLessScuffedAuton_RED.IntakeState.STALLING;
                        intakeTimer.reset();
                    }
                    break;
                case STALLING:
                    rightIntake.setPower(-intakePower);
                    if(intakeTimer.milliseconds()>200)
                    {
                        d_open.setPosition(d_open_clamp);
                    }
                    if (intakeTimer.milliseconds()>600)
                    {
                        rightIntake.setPower(0);
                        d_coverRight.setPosition(d_minRange_coverRight);

                        //if d_open has moved to its needed position
                        IState = EvenLessScuffedAuton_RED.IntakeState.EXTENDING_LIFT;
                        past_lift_value=10000;
                        time.reset();
                        lift_front.setTargetPosition(-alliance_targetTipped);
                        lift_back.setTargetPosition(-alliance_targetTipped);
                        lift_front.setMode(DcMotor.RunMode.RUN_TO_POSITION);   // Move to deposit position
                        lift_back.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    }
                    break;
                case EXTENDING_LIFT:
                    lift_front.setPower(-1.0);
                    lift_back.setPower(-1.0);
                    if (checkLift())
                    {
                        lift_front.setPower(0);
                        lift_back.setPower(0);
                        IState = EvenLessScuffedAuton_RED.IntakeState.DONE;
                    }
                    break;
                case DONE:
                    break;
            }
        }
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


    public void deposit()
    {

        d_bendLeft.setPosition(d_maxRange_bendLeft);
        d_bendRight.setPosition(d_maxRange_bendRight);

        // Open to deposit in top level of alliance hub

        //Thread.sleep(500);
        double target=SystemClock.uptimeMillis()+500;
        while(SystemClock.uptimeMillis()<target)
        {
            //stall
            d_open.setPosition(d_open_top);
        }

        // Close & bend down deposit
        d_open.setPosition(d_open_minRange);
        d_bendLeft.setPosition(d_minRange_bendLeft);
        d_bendRight.setPosition(d_minRange_bendRight);

        //Thread.sleep(1000);


        past_lift_value=10000;
        time.reset();
        // Retract arm to original position
        lift_front.setTargetPosition(0);
        lift_back.setTargetPosition(0);
        lift_front.setMode(DcMotor.RunMode.RUN_TO_POSITION);   // Move to deposit position
        lift_back.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        // Thread.sleep(1000);

        while(!checkLift())
        {
            lift_front.setPower(1.0);
            lift_back.setPower(1.0);
        }
        lift_front.setPower(0);
        lift_back.setPower(0);
        lift_front.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift_back.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        d_open.setPosition(d_open_minRange);
        d_bendLeft.setPosition(d_minRange_bendLeft);
        d_bendRight.setPosition(d_minRange_bendRight);

        d_coverLeft.setPosition(d_minRange_coverLeft);
        d_coverRight.setPosition(d_minRange_coverRight);
    }
    public void visionDeposit(int level)
    {
        if(level==1)
        {

        }
        if(level==2)
        {

        }
        if(level==3)
        {

        }
    }
    public void webcamInit()
    {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam"), cameraMonitorViewId);
        pipeline=new VisionPipeline(telemetry);
        camera.setPipeline(pipeline);
        camera.setMillisecondsPermissionTimeout(2500); // Timeout for obtaining permission is configurable. Set before opening.
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {
                telemetry.addData("Status:", "pain");
                telemetry.update();
            }
        });
    }
}
