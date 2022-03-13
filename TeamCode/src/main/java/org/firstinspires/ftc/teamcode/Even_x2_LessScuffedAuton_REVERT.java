package org.firstinspires.ftc.teamcode;

import android.os.SystemClock;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDriveCancelable;
import org.firstinspires.ftc.teamcode.vision.VisionPipeline_BLUE;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous
public class Even_x2_LessScuffedAuton_REVERT extends LinearOpMode
{
    //enum definitions
    enum GrabbingState {GETTING,
        RETURNING,
        HAS_CUBE,
        DONE
    }
    enum ReturningState
    {
        LING, //Ling arknights :)
        DONE
    }
    enum IntakeState
    {
        INTO_DEPOSIT, EXTENDING_LIFT_1, EXTENDING_LIFT_2, DONE
    }

    enum OverallState
    {
        RESET, GOING, RETURNING, DEPOSITING
    }
    //static variables for positions
    final static int LIFT_EXTENDED=330;

    //changing variables that are used for stuff
    int cubePos;
    double timeStamp1;
    double pathChange;
    double tempTarget;
    int gotLine;

    //instances of enums
    GrabbingState GState;
    ReturningState RState;
    IntakeState IState;
    OverallState OState;

    //Motors
    DcMotor leftIntake;
    DcMotor rightIntake;

    //timers
    ElapsedTime globalTimer;
    ElapsedTime latencyTimer;

    //camera stuff
    OpenCvWebcam camera;
    VisionPipeline_BLUE pipeline;

    //Dashboard stuff
    FtcDashboard dashboard;
    TelemetryPacket packet;

    //Other, custom objects
    SampleMecanumDriveCancelable drive;
    LiftAsync lift;
    SensorController sensorController;
    ServoControl servoControl;
    TrajectoryGen_REVERT trajGen;

    @Override
    public void runOpMode() throws InterruptedException
    {
        //it'll work first try hopium
        //akiha my beloved make my code work
        //my code bricks more than brick eyes white dragon
        leftIntake = hardwareMap.dcMotor.get("leftIntake");
        rightIntake = hardwareMap.dcMotor.get("rightIntake");

        OState=OverallState.RESET;
        gotLine =0;

        dashboard=FtcDashboard.getInstance();
        packet=new TelemetryPacket();

        drive=new SampleMecanumDriveCancelable(hardwareMap);
        lift=new LiftAsync(hardwareMap, 0);
        sensorController=new SensorController(hardwareMap, SensorController.Side.LEFT);
        servoControl=new ServoControl(hardwareMap, ServoControl.Side.LEFT);
        trajGen=new TrajectoryGen_REVERT(drive, 70);

        globalTimer=new ElapsedTime();
        latencyTimer=new ElapsedTime();

        Pose2d startPose=new Pose2d(0,0,0);
        drive.setPoseEstimate(startPose);

        //webcamInit();

        servoControl.returnArm();
        servoControl.raiseAllIntakes();
        servoControl.startingPos();

        waitForStart();

        globalTimer.reset();
        latencyTimer.reset();
        //cubePos= pipeline.getAnalysis();
        //camera.stopStreaming();
        //camera.closeCameraDevice();
        //drive.followTrajectory(trajGen.preTrajectory(-19.25));
        //drive.setPoseEstimate(new Pose2d(0, 0, 0));
        //visionDeposit(cubePos);

        while(opModeIsActive()&&globalTimer.seconds()<27)
        {
            latencyTimer.reset();
            switch(OState)
            {
                case RESET:
                    drive.setPoseEstimate(new Pose2d(0, 0, 0));
                    lift.setPosition(0, 0.8);
                    servoControl.startingPos();
                    servoControl.lowerIntakes();
                    pathChange=0;
                    OState=OverallState.GOING;
                    GState=GrabbingState.GETTING;

                    drive.followTrajectoryAsync(trajGen.firstGoingTrajectory(49, -1, -5, 65, -2.6, -7.4));
                    timeStamp1=latencyTimer.milliseconds();
                    break;

                case GOING:
                    switch(GState)
                    {
                        case GETTING:
                            leftIntake.setPower(1);
                            if(sensorController.hasBlock())
                            {
                                tempTarget= SystemClock.uptimeMillis()+250;
                                GState=GrabbingState.HAS_CUBE;
                                servoControl.raiseIntakes();
                                servoControl.openDepositIntake();
                            }
                            else if(!drive.isBusy())
                            {
                                pathChange++;
                                if(pathChange<6)
                                {
                                    GState = GrabbingState.RETURNING;
                                    drive.followTrajectoryAsync(trajGen.returningTrajectory(49, -1, -5));
                                }
                                else
                                {
                                    tempTarget= SystemClock.uptimeMillis();
                                    GState=GrabbingState.HAS_CUBE;
                                    servoControl.raiseIntakes();
                                }
                            }
                            break;
                        case RETURNING:
                            if(!drive.isBusy())
                            {
                                GState = GrabbingState.GETTING;
                                drive.followTrajectoryAsync(trajGen.goingTrajectory(73 + 2 * pathChange, -3.1 - pathChange, Math.toRadians(-7.8 - (2.5 * pathChange))));
                            }
                            break;
                        case HAS_CUBE:
                            if(SystemClock.uptimeMillis()<tempTarget)
                            {
                                leftIntake.setPower(0.7);
                            }
                            else
                            {
                                drive.cancelFollowing();
                                leftIntake.setPower(0);
                                sensorController.closeIntakeSensor();
                                servoControl.openDepositIntake();
                                IState=IntakeState.INTO_DEPOSIT;
                                drive.followTrajectoryAsync(trajGen.realReturnTrajectory());
                                GState=GrabbingState.DONE;
                                OState=OverallState.RETURNING;
                                RState=ReturningState.LING;
                                tempTarget=SystemClock.uptimeMillis()+750;
                            }
                            break;
                        case DONE:
                            break;
                    }
                    if(lift.getPos2()<80)
                    {
                        servoControl.returnArm();
                    }
                    break;
                case RETURNING:
                    switch(RState)
                    {
                        case LING:
                            if(sensorController.onColor())
                            {
                                gotLine++;
                                drive.setPoseEstimate(new Pose2d(33.5, 7, drive.getPoseEstimate().getHeading()));
                            }
                            if(!drive.isBusy()&&IState==IntakeState.DONE)
                            {
                                OState=OverallState.DEPOSITING;
                                RState=ReturningState.DONE;
                                servoControl.openDeposit();
                            }
                            break;
                        case DONE:
                            break;
                    }
                    switch(IState)
                    {
                        case INTO_DEPOSIT:
                            if(SystemClock.uptimeMillis()>tempTarget)
                            {
                                leftIntake.setPower(-1);
                            }
                            else
                            {
                                leftIntake.setPower(1);
                            }
                            if(sensorController.depositCube())
                            {
                                servoControl.prepDeposit();
                                leftIntake.setPower(0);
                                lift.setPosition(LIFT_EXTENDED);
                                servoControl.flipOut();
                                IState=IntakeState.EXTENDING_LIFT_2;
                            }
                            break;
                    /*case EXTENDING_LIFT_1:
                        if(lift.getPos2()>100)
                        {
                            servoControl.openTurret();
                            IState=IntakeState.EXTENDING_LIFT_2;
                        }
                        break;*/
                        case EXTENDING_LIFT_2:
                            if(!lift.isBusy())
                            {
                                IState=IntakeState.DONE;
                            }
                            break;
                        case DONE:
                            break;
                    }
                    break;
                case DEPOSITING:
                    servoControl.openDeposit();
                    if(sensorController.depositNoCube())
                    {
                        OState=OverallState.RESET;
                    }
                    break;
            }
            packet.put("OState ", OState);
            packet.put("GState ", GState);
            packet.put("RState ", RState);
            packet.put("IState ", IState);
            packet.put("Line", gotLine);
            packet.put("latency", latencyTimer.milliseconds());
            packet.put("Help", timeStamp1);

            packet.put("lift pos 1", lift.getPos1());
            packet.put("lift pos 2", lift.getPos2());
            packet.put("lift power", lift.getPower());
            dashboard.sendTelemetryPacket(packet);
            lift.adjustLift();
            drive.update();
        }

        lift.setPosition(0);
        servoControl.raiseAllIntakes();
        servoControl.startingPos();
        Trajectory park=drive.trajectoryBuilder(drive.getPoseEstimate())
                .lineToSplineHeading(new Pose2d(60, drive.getPoseEstimate().getY(), Math.toRadians(0)))
                .build();
        drive.followTrajectoryAsync(park);
        while(opModeIsActive()&&globalTimer.milliseconds()<29500&&(drive.isBusy()||lift.isBusy()))
        {
            lift.adjustLift();
            drive.update();
        }
        drive.cancelFollowing();
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
                camera.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {
                //telemetry.addData("Status:", "pain");
                //telemetry.update();
            }
        });
    }

    public void visionDeposit(int level)
    {
        if(level==1)
        {

        }
        else if(level==2)
        {

        }
        else if(level==3)
        {
            servoControl.prepDeposit();
            servoControl.flipMedium();
            lift.setPosition(380);
            while(lift.isBusy())
            {
                if(lift.getPos2()>100)
                {
                    servoControl.openTurret();
                }
            }
            servoControl.openDeposit();
            lift.brake();

            double tempTarget=SystemClock.uptimeMillis()+500;
            while(SystemClock.uptimeMillis()<tempTarget)
            {
                //stall
            }
        }
        //vision deposit code here
    }
}
