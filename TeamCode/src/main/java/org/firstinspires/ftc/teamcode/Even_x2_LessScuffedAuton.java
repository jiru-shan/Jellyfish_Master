package org.firstinspires.ftc.teamcode;

import android.os.SystemClock;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDriveCancelable;
import org.firstinspires.ftc.teamcode.vision.VisionPipeline_BLUE;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

public class Even_x2_LessScuffedAuton extends OpMode
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
        INTO_DEPOSIT, EXTENDING_LIFT, DONE
    }

    enum OverallState
    {
        RESET, GOING, RETURNING, DEPOSITING
    }
    //static variables for positions
    final static int LIFT_EXTENDED=0;

    //changing variables that are used for stuff
    int cubePos;
    boolean askStop;
    double pathChange;
    double tempTarget;

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

    //camera stuff
    OpenCvWebcam camera;
    VisionPipeline_BLUE pipeline;

    //Other, custom objects
    SampleMecanumDriveCancelable drive;
    LiftAsync lift;
    SensorController sensorController;
    ServoControl servoControl;
    TrajectoryGen trajGen;

    @Override
    public void init()
    {
        leftIntake = hardwareMap.dcMotor.get("leftIntake");
        rightIntake = hardwareMap.dcMotor.get("rightIntake");

        OState=OverallState.RESET;

        drive=new SampleMecanumDriveCancelable(hardwareMap);
        lift=new LiftAsync(hardwareMap, 0);
        sensorController=new SensorController(hardwareMap);
        servoControl=new ServoControl(hardwareMap, ServoControl.Side.LEFT);
        trajGen=new TrajectoryGen(drive, 40);

        globalTimer=new ElapsedTime();

        Pose2d startPose=new Pose2d(0,0,0);
        drive.setPoseEstimate(startPose);

        webcamInit();

        servoControl.startingPos();
    }

    @Override
    public void start()
    {
        globalTimer.reset();
        cubePos= pipeline.getAnalysis();

        drive.followTrajectory(trajGen.preTrajectory(-19.25));
        drive.setPoseEstimate(new Pose2d(0, 0, 0));

        visionDeposit(cubePos);
    }
    @Override
    public void loop()
    {
        if(globalTimer.seconds()>27)
        {
            askStop=true;
            requestOpModeStop();
        }
        switch(OState)
        {
            case RESET:
                drive.setPoseEstimate(new Pose2d(0, 0, 0));
                lift.setPosition(0);
                servoControl.startingPos();
                servoControl.lowerIntakes();
                pathChange=0;
                OState=OverallState.GOING;
                GState=GrabbingState.GETTING;
                drive.followTrajectoryAsync(trajGen.firstGoingTrajectory(49, -1, -5, 73, -2.25, -10));
                break;

            case GOING:

                switch(GState)
                {
                    case GETTING:
                        leftIntake.setPower(1);
                        if(sensorController.hasBlock())
                        {
                            tempTarget= SystemClock.uptimeMillis()+300;
                            GState=GrabbingState.HAS_CUBE;
                            servoControl.raiseIntakes();
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
                                drive.followTrajectoryAsync(trajGen.goingTrajectory(73 + 2 * pathChange, -2.25 - pathChange, Math.toRadians(-10 - (2.5 * pathChange))));
                        }
                        break;
                    case HAS_CUBE:
                        if(SystemClock.uptimeMillis()<tempTarget)
                        {
                            leftIntake.setPower(1);
                        }
                        else
                            {
                                drive.cancelFollowing();
                                leftIntake.setPower(0);
                                servoControl.openDeposit();
                                IState=IntakeState.INTO_DEPOSIT;
                                drive.followTrajectoryAsync(trajGen.realReturnTrajectory(49, 13, 0));
                                GState=GrabbingState.DONE;
                                OState=OverallState.RETURNING;
                                RState=ReturningState.LING;
                            }
                        break;
                    case DONE:
                        break;

                }
            break;
            case RETURNING:
                switch(RState)
                {
                    case LING:
                        if(sensorController.onColor())
                        {
                            drive.setPoseEstimate(new Pose2d(31.5, 0, 0));
                        }
                        if(!drive.isBusy()&&IState==IntakeState.DONE)
                        {
                            OState=OverallState.DEPOSITING;
                            RState=ReturningState.DONE;
                            servoControl.openDeposit();
                            tempTarget=SystemClock.uptimeMillis()+500;
                        }
                        break;
                    case DONE:
                        break;
                }
                switch(IState)
                {
                    case INTO_DEPOSIT:
                        leftIntake.setPower(-1);
                        if(sensorController.depositCube())
                        {
                            servoControl.closeDeposit();
                            leftIntake.setPower(0);
                            lift.setPosition(LIFT_EXTENDED);
                            servoControl.flipOut();
                            IState=IntakeState.EXTENDING_LIFT;
                        }
                        break;
                    case EXTENDING_LIFT:
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
                if(SystemClock.uptimeMillis()>tempTarget)
                {
                    OState=OverallState.RESET;
                }
                break;

        }
        drive.update();
        lift.adjustLift();
    }

    @Override
    public void stop()
    {
        if(askStop)
        {
            lift.setPosition(0);
            servoControl.raiseIntakes();
            servoControl.startingPos();
            Trajectory park=drive.trajectoryBuilder(drive.getPoseEstimate())
                    .lineToSplineHeading(new Pose2d(60, 0, Math.toRadians(0)))
                    .build();
            drive.followTrajectoryAsync(park);
            while(globalTimer.milliseconds()<29500)
            {
                lift.adjustLift();
                drive.update();
            }
            drive.cancelFollowing();
        }
        else
            {
                //die
            }
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
                telemetry.addData("Status:", "pain");
                telemetry.update();
            }
        });
    }

    public void visionDeposit(int level)
    {

        //vision deposit code here
    }
}
