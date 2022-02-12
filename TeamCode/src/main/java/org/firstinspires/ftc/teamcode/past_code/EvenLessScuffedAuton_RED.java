package org.firstinspires.ftc.teamcode.past_code;

import android.os.SystemClock;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
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
import org.firstinspires.ftc.teamcode.vision.VisionPipeline_RED;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous
@Config
public class EvenLessScuffedAuton_RED extends LinearOpMode
{
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
    Rev2mDistanceSensor depositSensor;

    //APPROACH 1: Start the deposit low so weighted cubes can make it in and then
    //increase the servo values so that deposit climbs higher for ball

    //APPROACH 2: Wide deposit and then clamp as soon as it stalls


    // Deposit servo positions
    double d_open_minRange = 0.59;
    public static double d_open_top = 0.4;
    public static double d_open_clamp=0.66;
    public static double d_minRange_bendLeft = 0.96;      // need to fix bend values
    public static double d_minRange_bendRight = 0.04;


    double i_minRange_topRight = 0.96;
    double i_maxRange_topRight = 0.20;
    double i_minRange_bottomRight = 0.04;
    double i_maxRange_bottomRight = 0.80;

    double d_minRange_coverLeft = 0.55;
    double d_minRange_coverRight = 0.45;
    double d_maxRange_coverRight = 0.85;

    public static double d_bendLeft_lift=0.8;
    public static double d_bendRight_lift=0.2;

    double intakePower=1;

    OpenCvWebcam camera;
    VisionPipeline_RED pipeline;

    double past_lift_value;
    double CYCLE_TIME=8;
    int cubeCheck;
    boolean timeCheck;
    int cycleAmount;

    ElapsedTime time = new ElapsedTime();
    ElapsedTime intakeTimer=new ElapsedTime();
    ElapsedTime overallTime=new ElapsedTime();

    enum GrabbingState {GETTING,
        RETURNING,
        HAS_CUBE,
        HAS_CUBE_2,
        DONE
    }

    enum ReturningState
    {
        LING, //Ling arknights :)
        RETURNING,
        DONE
    }
    enum IntakeState
    {
        INTO_DEPOSIT, STALLING, EXTENDING_LIFT, DONE
    }

    int alliance_targetTipped = 625;

    SampleMecanumDriveCancelable drive;

    int cubePos;

    Trajectory goingTrajectory;
    Trajectory returningTrajectory;
    GrabbingState GState;
    ReturningState RState;
    IntakeState IState;

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
        depositSensor=hardwareMap.get(Rev2mDistanceSensor.class, "depositSensor");

        drive = new SampleMecanumDriveCancelable(hardwareMap);
        Pose2d startPose = new Pose2d(0, 0, 0);
        ElapsedTime time = new ElapsedTime();
        drive.setPoseEstimate(startPose);

        webcamInit();

        d_coverLeft.setPosition(d_minRange_coverLeft);
        d_coverRight.setPosition(d_minRange_coverRight);
        d_open.setPosition(d_open_clamp);
        d_bendLeft.setPosition(d_minRange_bendLeft);
        d_bendRight.setPosition(d_minRange_bendRight);



        i_bottomRight.setDirection(Servo.Direction.FORWARD);
        i_topRight.setDirection(Servo.Direction.FORWARD);

        waitForStart();
        cubePos= pipeline.getAnalysis();

        Trajectory prepreTrajectory = drive.trajectoryBuilder(startPose)
                .forward(21.5+7)
                .build();
        drive.followTrajectory(prepreTrajectory);
        drive.setPoseEstimate(new Pose2d(0, 0, Math.toRadians(0)));

        //Run roadrunner code to place in correct level and then return to starting pos
        visionDeposit(cubePos);

        while (cycleAmount<2)
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
            cubeCheck=0;
            pathChange = 1;
            while (GState!=GrabbingState.DONE && pathChange < 3)
            {
                switch (GState) {
                    case GETTING:
                        if(hasBlock())
                        {
                            GState= GrabbingState.HAS_CUBE;
                        }
                        else if (!drive.isBusy()) {

                            GState = GrabbingState.RETURNING;
                            returningTrajectory = drive.trajectoryBuilder(drive.getPoseEstimate())
                                    .lineToSplineHeading(new Pose2d(-49, -1, Math.toRadians(5)))
                                    .build();
                            drive.followTrajectoryAsync(returningTrajectory);
                        }
                        break;
                    case RETURNING:
                        if (!drive.isBusy()) {
                            GState = GrabbingState.GETTING;
                            goingTrajectory = drive.trajectoryBuilder(drive.getPoseEstimate())
                                    .lineToSplineHeading(new Pose2d(-73 - 2 * pathChange, -2.25 - pathChange, Math.toRadians(10 + (2.5 * pathChange))), SampleMecanumDrive.getVelocityConstraint(40,
                                            DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                                    .build();
                            drive.followTrajectory(goingTrajectory);
                            pathChange++;
                        }
                        break;
                    case HAS_CUBE:
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
                                .lineToSplineHeading(new Pose2d(-49, 15, Math.toRadians(0)))
                                .build();
                        drive.followTrajectoryAsync(returningTrajectory);
                        GState=GrabbingState.HAS_CUBE_2;

                        break;
                    case HAS_CUBE_2:
                        if(hasBlock())
                        {
                            cubeCheck+=3;
                        }
                        else if(!hasBlock())
                        {
                            cubeCheck--;
                        }
                        if(!drive.isBusy())
                        {
                            drive.setPoseEstimate(new Pose2d(drive.getPoseEstimate().getX(), 0, 0));
                            if(cubeCheck>0)
                            {
                                GState=GrabbingState.DONE;
                            }
                            else if(cubeCheck<0)
                                {
                                    GState=GrabbingState.RETURNING;
                                }
                        }
                        break;
                    case DONE:
                        break;
                }
                drive.update();
                if(GState!=GrabbingState.HAS_CUBE_2)
                {
                    rightIntake.setPower(1);
                }
            }

            rightIntake.setPower(0);

            RState = ReturningState.LING;
            IState=IntakeState.INTO_DEPOSIT;
            //intakeTarget=SystemClock.uptimeMillis()+2500;
            intakeTimer.reset();
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
                            drive.setPoseEstimate(new Pose2d(-40.5, 0, 0));
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
            cycleAmount++;
       }
            Trajectory park = drive.trajectoryBuilder(drive.getPoseEstimate())
                    .lineToSplineHeading(new Pose2d(-60, 0, Math.toRadians(0)))
                    .build();
            drive.followTrajectory(park);
    }
    public boolean checkLift()
    {
        telemetry.addData(">", lift_front.getCurrentPosition());
        telemetry.update();
        double change=727727727;
        if(time.milliseconds()>500)
        {
            change=Math.abs(Math.abs(lift_front.getCurrentPosition())-Math.abs(past_lift_value));
            past_lift_value=lift_front.getCurrentPosition();
            time.reset();
        }
        if(change<6)
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
                    if (depositCube())
                    {
                        IState = EvenLessScuffedAuton_RED.IntakeState.STALLING;
                        intakeTimer.reset();
                    }
                    break;
                case STALLING:
                    rightIntake.setPower(-intakePower);
                    if(intakeTimer.milliseconds()>50)
                    {
                        d_open.setPosition(d_open_clamp);
                    }
                    if (intakeTimer.milliseconds()>1500)
                    {
                        rightIntake.setPower(0);
                        d_coverRight.setPosition(d_minRange_coverRight);

                        //if d_open has moved to its needed position
                        IState = EvenLessScuffedAuton_RED.IntakeState.EXTENDING_LIFT;
                        past_lift_value=10000;
                        time.reset();
                        d_bendLeft.setPosition(d_bendLeft_lift);
                        d_bendRight.setPosition(d_bendRight_lift);
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


    public void checkForExit()
    {
        if(overallTime.seconds()>=26)
        {
            timeCheck=false;
            d_coverLeft.setPosition(d_minRange_coverLeft);
            d_coverRight.setPosition(d_minRange_coverRight);
            d_open.setPosition(d_open_minRange);
            d_bendLeft.setPosition(d_minRange_bendLeft);
            d_bendRight.setPosition(d_minRange_bendRight);
            drive.cancelFollowing();
            Trajectory park=drive.trajectoryBuilder(drive.getPoseEstimate())
                    .lineToSplineHeading(new Pose2d(-60, 0, Math.toRadians(0)))
                    .build();
            drive.followTrajectoryAsync(park);
            lift_back.setTargetPosition(0);
            lift_front.setTargetPosition(0);
            lift_back.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            lift_front.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            while(!checkLift())
            {
                lift_front.setPower(1);
                lift_back.setPower(1);
                drive.update();
            }
        }
    }
    public void deposit()
    {

        //d_bendLeft.setPosition(d_maxRange_bendLeft);
        //d_bendRight.setPosition(d_maxRange_bendRight);

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
            Trajectory strafeTrajectory = drive.trajectoryBuilder(new Pose2d())
                    .strafeRight(25)
                    .build();
            drive.followTrajectory(strafeTrajectory);
            lift_front.setTargetPosition(-200);
            lift_back.setTargetPosition(-200);
            lift_front.setMode(DcMotor.RunMode.RUN_TO_POSITION);   // Move to deposit position
            lift_back.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            d_bendLeft.setPosition(0.8);
            d_bendRight.setPosition(0.2);
            time.reset();
            while(!checkLift())
            {
                lift_front.setPower(-1.0);
                lift_back.setPower(-1.0);
            }
            double target=SystemClock.uptimeMillis()+500;
            while(SystemClock.uptimeMillis()<target)
            {
                //stall
                d_open.setPosition(0.15);
            }
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

            Trajectory strafeBack=drive.trajectoryBuilder(drive.getPoseEstimate())
            .lineToConstantHeading(new Vector2d(0,3))
                    .build();

            drive.followTrajectory(strafeBack);

        }
        if(level==2)
        {
            d_open.setPosition(d_open_clamp);
            lift_front.setTargetPosition(-(alliance_targetTipped-200));
            lift_back.setTargetPosition(-(alliance_targetTipped-200));
            lift_front.setMode(DcMotor.RunMode.RUN_TO_POSITION);   // Move to deposit position
            lift_back.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            d_bendLeft.setPosition(0.6);
            d_bendRight.setPosition(0.4);
            time.reset();
            while(!checkLift())
            {
                lift_front.setPower(-1.0);
                lift_back.setPower(-1.0);
            }
            double target=SystemClock.uptimeMillis()+500;
            while(SystemClock.uptimeMillis()<target)
            {
                //stall
                d_open.setPosition(0.3);
            }
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
        if(level==3)
        {
            d_open.setPosition(d_open_clamp);
            lift_front.setTargetPosition(-alliance_targetTipped);
            lift_back.setTargetPosition(-alliance_targetTipped);
            lift_front.setMode(DcMotor.RunMode.RUN_TO_POSITION);   // Move to deposit position
            lift_back.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            d_bendLeft.setPosition(d_bendLeft_lift);
            d_bendRight.setPosition(d_bendRight_lift);
            time.reset();
            while(!checkLift())
            {
                lift_front.setPower(-1.0);
                lift_back.setPower(-1.0);
            }
            lift_front.setPower(0);
            lift_back.setPower(0);

            deposit();
        }
    }
    public boolean depositCube()
    {
        if(depositSensor.getDistance(DistanceUnit.CM)<13)
        {
            return true;
        }
        return false;
    }
    public void webcamInit()
    {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam"), cameraMonitorViewId);
        pipeline=new VisionPipeline_RED(telemetry);
        camera.setPipeline(pipeline);
        camera.setMillisecondsPermissionTimeout(2500); // Timeout for obtaining permission is configurable. Set before opening.
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
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
