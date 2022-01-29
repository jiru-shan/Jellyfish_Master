package org.firstinspires.ftc.teamcode;

import android.os.SystemClock;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
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
import org.firstinspires.ftc.teamcode.vision.VisionPipeline_BLUE;
import org.firstinspires.ftc.teamcode.vision.VisionPipeline_RED;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous
@Config
public class EvenLessScuffedAuton_BLUE extends LinearOpMode
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


    // Deposit servo positions
    double d_open_minRange = 0.65; // it was 0.59
    double d_open_top = 0.4;
    double d_open_clamp=0.66;
    double d_minRange_bendLeft = 0.96;      // need to fix bend values
    double d_minRange_bendRight = 0.04;
    public static double d_bendLeft_lift=0.8;
    public static double d_bendRight_lift=0.2;


    double i_minRange_topLeft = 0.1;
    double i_maxRange_topLeft = 0.85;
    double i_minRange_bottomLeft = 0.9;
    double i_maxRange_bottomLeft = 0.15;

    double d_minRange_coverLeft = 0.55;
    double d_maxRange_coverLeft = 0.15;
    double d_minRange_coverRight = 0.45;

    double prevTimer;
    ElapsedTime intakeTimer=new ElapsedTime();

    OpenCvWebcam camera;
    VisionPipeline_BLUE pipeline;

    double CYCLE_TIME=8;
    double past_lift_value;
    boolean cubeCheck;

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
        INTO_DEPOSIT, STALLING, EXTENDING_LIFT,  DONE
    }

    int alliance_targetTipped = 1000;
    double intakeTarget=0;
    double intakeEjectDistance=30.0;

    SampleMecanumDriveCancelable drive;

    Trajectory goingTrajectory;
    Trajectory returningTrajectory;
    GrabbingState GState;
    ReturningState RState;
    IntakeState IState;

    ElapsedTime time = new ElapsedTime();
    ElapsedTime overallTime=new ElapsedTime();


    int cubePos;

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

        leftFront.setDirection(DcMotorSimple.Direction.FORWARD);
        leftBack.setDirection(DcMotorSimple.Direction.FORWARD);
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
        depositSensor=hardwareMap.get(Rev2mDistanceSensor.class, "depositSensor");


        drive = new SampleMecanumDriveCancelable(hardwareMap);
        Pose2d startPose = new Pose2d(0, 0, 0);
        drive.setPoseEstimate(startPose);

        webcamInit();

        d_open.setPosition(d_open_clamp);
        d_bendLeft.setPosition(d_minRange_bendLeft);
        d_bendRight.setPosition(d_minRange_bendRight);

        d_coverLeft.setPosition(d_minRange_coverLeft);
        d_coverRight.setPosition(d_minRange_coverRight);
        d_open.setPosition(d_open_minRange);
        d_open.setPosition(d_open_minRange);
        d_open.setPosition(d_open_minRange);
        d_open.setPosition(d_open_minRange);
        d_open.setPosition(d_open_minRange);
        d_open.setPosition(d_open_minRange);

        waitForStart();

        cubePos= pipeline.getAnalysis();

        Trajectory prepreTrajectory = drive.trajectoryBuilder(startPose)
                .back(7+12.25)
                .build();
        drive.followTrajectory(prepreTrajectory);
        drive.setPoseEstimate(new Pose2d(0, 0, Math.toRadians(0)));

        //run roadrunner code to deposit in correct place and then return to starting pos
        visionDeposit(cubePos);

        while(30-overallTime.seconds()>CYCLE_TIME+3)
        {
        drive.setPoseEstimate(new Pose2d(0, 0, Math.toRadians(0)));
        lowerIntakes();
            d_bendLeft.setPosition(d_minRange_bendLeft);
            d_bendRight.setPosition(d_minRange_bendRight);
            d_open.setPosition(d_open_minRange);
        double pathChange=0;
        GState = GrabbingState.GETTING;
        cubeCheck=false;
        goingTrajectory = drive.trajectoryBuilder(startPose)

                .splineTo(new Vector2d(49, -1), Math.toRadians(-5))
                .splineTo(new Vector2d(73, -2.25-pathChange), Math.toRadians(-10-(2.5*pathChange)), SampleMecanumDrive.getVelocityConstraint(40,
                        DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                //.lineToSplineHeading(new Pose2d(73, -2.25, Math.toRadians(-10)))
                .build();
        drive.followTrajectoryAsync(goingTrajectory);
        pathChange=1;
        while(GState!= EvenLessScuffedAuton_BLUE.GrabbingState.DONE && pathChange < 3)
        {
            switch(GState)
            {
                case GETTING:
                    if(hasBlock())
                    {
                        GState= EvenLessScuffedAuton_BLUE.GrabbingState.HAS_CUBE;
                    }
                    else if(!drive.isBusy())
                    {

                        GState = GrabbingState.RETURNING;
                        returningTrajectory= drive.trajectoryBuilder(drive.getPoseEstimate())
                                .lineToSplineHeading(new Pose2d(49, -1, Math.toRadians(-5)))
                                .build();
                        drive.followTrajectoryAsync(returningTrajectory);
                    }
                    break;
                case RETURNING:
                    if(!drive.isBusy())
                    {
                        GState = GrabbingState.GETTING;
                        goingTrajectory=drive.trajectoryBuilder(drive.getPoseEstimate())
                                .splineTo(new Vector2d(73+2*pathChange, -2.25-pathChange), Math.toRadians(-10-(2.5*pathChange)), SampleMecanumDrive.getVelocityConstraint(40,
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
                        leftIntake.setPower(1);
                        drive.update();
                        raiseIntakes();
                    }
                    drive.cancelFollowing();
                    leftIntake.setPower(0);
                    returningTrajectory = drive.trajectoryBuilder(drive.getPoseEstimate())
                            .lineToSplineHeading(new Pose2d(49, 8, Math.toRadians(0)))
                            .build();
                    drive.followTrajectoryAsync(returningTrajectory);
                    GState= EvenLessScuffedAuton_BLUE.GrabbingState.HAS_CUBE_2;

                    break;
                case HAS_CUBE_2:
                    if(hasBlock())
                    {
                        cubeCheck=true;
                    }
                    if(!drive.isBusy())
                    {
                        drive.setPoseEstimate(new Pose2d(drive.getPoseEstimate().getX(), 0, 0));
                        if(cubeCheck)
                        {
                            GState= EvenLessScuffedAuton_BLUE.GrabbingState.DONE;
                        }
                        else
                        {
                            GState= EvenLessScuffedAuton_BLUE.GrabbingState.RETURNING;
                        }
                    }
            }
            drive.update();
            if(GState!= EvenLessScuffedAuton_BLUE.GrabbingState.HAS_CUBE_2)
            {
                leftIntake.setPower(1);
            }
        }

        leftIntake.setPower(0);

        RState = ReturningState.LING;
        IState=IntakeState.INTO_DEPOSIT;
        intakeTimer.reset();
        prevTimer=100;
        d_open.setPosition(d_open_minRange);
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
                        drive.setPoseEstimate(new Pose2d(31.5, 0, 0));
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
                    if (!drive.isBusy()&&IState==IntakeState.DONE)
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
        Trajectory park=drive.trajectoryBuilder(drive.getPoseEstimate())
                .lineToSplineHeading(new Pose2d(50, 0, Math.toRadians(0)))
                .build();
        drive.followTrajectory(park);
    }
    public void raiseIntakes()
    {
        i_topLeft.setPosition(i_maxRange_topLeft);
        i_bottomLeft.setPosition(i_maxRange_bottomLeft);
    }
    public void lowerIntakes()
    {
        i_topLeft.setPosition(i_minRange_topLeft);
        i_bottomLeft.setPosition(i_minRange_bottomLeft);
    }

    public boolean onColor()
    {
        if(rgbAvg(driveLeft)>175&&rgbAvg(driveRight)>175)
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

    public void putInDeposit()
    {
        if(IState!=IntakeState.DONE)
        {
            switch (IState) {
                case INTO_DEPOSIT:
                    d_open.setPosition(d_open_minRange);
                    d_bendLeft.setPosition(d_minRange_bendLeft);
                    d_bendRight.setPosition(d_minRange_bendRight);
                    d_coverLeft.setPosition(d_maxRange_coverLeft);
                    d_coverRight.setPosition(d_minRange_coverRight);
                    leftIntake.setPower(-1);
                    if (depositCube()) {
                        IState = IntakeState.STALLING;
                        intakeTimer.reset();
                    }
                    break;
                case STALLING:
                    leftIntake.setPower(-1);
                    if(intakeTimer.milliseconds()>50) {
                        d_open.setPosition(d_open_clamp);
                    }
                    if (intakeTimer.milliseconds()>1500) {
                        leftIntake.setPower(0);
                        d_coverLeft.setPosition(d_minRange_coverLeft);
                        IState = IntakeState.EXTENDING_LIFT;
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
                    //pain
                    lift_front.setPower(-1.0);
                    lift_back.setPower(-1.0);
                    if (checkLift())
                    {
                        lift_front.setPower(0);
                        lift_back.setPower(0);
                        IState = IntakeState.DONE;
                    }
                    break;
                case DONE:
                    break;
            }
        }
    }
    public void deposit()
    {
        //d_bendLeft.setPosition(d_maxRange_bendLeft);
        //d_bendRight.setPosition(d_maxRange_bendRight);

        // Open to deposit in top level of alliance hub

        double target=SystemClock.uptimeMillis()+500;
        while(SystemClock.uptimeMillis()<target)
        {
            //stall
            d_open.setPosition(d_open_top);
        }
        //Thread.sleep(500);


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
        pipeline=new VisionPipeline_BLUE(telemetry);
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
    public void visionDeposit(int level)
    {
        if(level==1)
        {
            Trajectory strafeTrajectory = drive.trajectoryBuilder(new Pose2d())
                    .strafeRight(28)
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
                    .lineToConstantHeading(new Vector2d(0,0))
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
}
