package org.firstinspires.ftc.teamcode;

import android.os.SystemClock;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDriveCancelable;
import org.firstinspires.ftc.teamcode.vision.VisionPipeline_BLUE;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous
public class Even_x2_LessScuffedAuton extends LinearOpMode
{
    //enum definitions
    enum GrabbingState {GETTING,
        RETURNING,
        HAS_CUBE,
        DONE
    }
    enum ReturningState
    {
        TURNING,
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
    final static int LIFT_EXTENDED=340;
    final static int LIFT_MIDDLE=430;

    //changing variables that are used for stuff
    int cubePos;
    double timeStamp1;
    double pathChangeTemp;
    double pathChangeReal;
    double tempTarget;
    int gotLine;
    double cutOff;
    double intakeCounter;
    boolean offset;

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
    TrajectoryGen trajGen;

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
        cutOff=3;
        pathChangeReal=0;

        dashboard=FtcDashboard.getInstance();
        packet=new TelemetryPacket();

        drive=new SampleMecanumDriveCancelable(hardwareMap);
        lift=new LiftAsync(hardwareMap, 0);
        sensorController=new SensorController(hardwareMap, SensorController.Side.LEFT);
        servoControl=new ServoControl(hardwareMap, ServoControl.Side.LEFT);
        trajGen=new TrajectoryGen(drive, /*35*/35);

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

        while(opModeIsActive()&&globalTimer.seconds()<30-cutOff)
        {
            latencyTimer.reset();
            switch(OState)
            {
                case RESET:
                    drive.setPoseEstimate(new Pose2d(0, 0, 0));
                    lift.setPosition(0, 0.8);
                    servoControl.startingPos();
                    servoControl.lowerIntakes();
                    pathChangeTemp =0;
                    offset=false;
                    OState=OverallState.GOING;
                    GState=GrabbingState.GETTING;
                    IState=IntakeState.INTO_DEPOSIT;

                    //54 for x1, and 3.5 for increment on ang 2
                    drive.followTrajectoryAsync(trajGen.firstGoingTrajectory(51+pathChangeReal*2, -1, -3, 65+pathChangeReal*2.5, -1.6-(0.5*pathChangeReal), -8.4-(3*pathChangeReal), false));
                    //drive.followTrajectoryAsync(trajGen.firstGoingTrajectory(45, -0.25, -1, 73, -0.9-(0.2*pathChangeReal), -2.5-(0.5*pathChangeReal), false));
                    timeStamp1=latencyTimer.milliseconds();
                    break;

                case GOING:
                    double temp=sensorController.intakeDistance();
                    switch(GState)
                    {
                        case GETTING:
                            leftIntake.setPower(1);
                            if(temp<10)
                            {
                                drive.cancelFollowing();
                                //drive.setMotorPowers(0, 0, 0, 0);
                                //drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                                tempTarget= SystemClock.uptimeMillis()+25; //150 //250
                                GState=GrabbingState.HAS_CUBE;
                                servoControl.raiseIntakes();
                                servoControl.openDepositIntake();
                            }
                            else if(!drive.isBusy())
                            {
                                pathChangeTemp++;
                                if(pathChangeTemp <6)
                                {
                                    GState = GrabbingState.RETURNING;
                                    drive.followTrajectoryAsync(trajGen.returningTrajectory(46, -1, -5));
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
                            leftIntake.setPower(1);
                            if(temp<15)
                            {
                                tempTarget= SystemClock.uptimeMillis()+100;
                                GState=GrabbingState.HAS_CUBE;
                                servoControl.raiseIntakes();
                                servoControl.openDepositIntake();
                            }
                            if(!drive.isBusy())
                            {
                                GState = GrabbingState.GETTING;
                                drive.followTrajectoryAsync(trajGen.goingTrajectory(73 + 2 * pathChangeTemp, -3.1 - pathChangeTemp, Math.toRadians(-7.8 - (2.5 * pathChangeTemp))));
                            }
                            break;
                        case HAS_CUBE:
                            if(SystemClock.uptimeMillis()<tempTarget)
                            {
                                if(temp>25)
                                {
                                    GState=GrabbingState.GETTING;
                                }
                                leftIntake.setPower(0.3);
                            }
                            else
                            {
                                    drive.cancelFollowing();
                                    //drive.setMotorPowers(0, 0, 0, 0);
                                    //drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                                    leftIntake.setPower(0.4);
                                    sensorController.closeIntakeSensor();
                                    IState = IntakeState.INTO_DEPOSIT;
                                    //drive.setPoseEstimate(new Pose2d(drive.getPoseEstimate().getX()-25, drive.getPoseEstimate().getY(), drive.getPoseEstimate().getHeading()));
                                    //offset=true;
                                    //drive.followTrajectoryAsync(trajGen.realReturnTrajectory());
                                    GState = GrabbingState.DONE;
                                    OState = OverallState.RETURNING;
                                    //RState = ReturningState.LING;
                                    RState=ReturningState.LING;
                                    //drive.setPoseEstimate(new Pose2d(drive.getPoseEstimate().getX(), drive.getPoseEstimate().getY(), drive.getRawExternalHeadingDegrees()-360));
                                    servoControl.openDepositIntake();
                                    servoControl.raiseIntakes();
                                    drive.followTrajectoryAsync(trajGen.realReturnTrajectory());
                                    tempTarget = SystemClock.uptimeMillis() + 500;
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
                               drive.setPoseEstimate(new Pose2d(32.8, 7, drive.getPoseEstimate().getHeading()));
                            }
                            if(!drive.isBusy2()&&IState==IntakeState.DONE)
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
                                leftIntake.setPower(0.3);
                            }
                            if(sensorController.depositCube())
                            {
                                servoControl.prepDeposit();
                                leftIntake.setPower(0);
                                lift.setPosition(LIFT_EXTENDED+(int)(pathChangeReal*3));
                                servoControl.flipOut();
                                IState=IntakeState.EXTENDING_LIFT_1;
                            }
                            if(!drive.isBusy2())
                            {
                                leftIntake.setPower(0);
                                IState=IntakeState.DONE;
                            }
                            break;
                    case EXTENDING_LIFT_1:
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
                        pathChangeReal++;
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
            //packet.put("mental damage", drive.getWheelVelocities());

            packet.put("lift pos 1", lift.getPos1());
            packet.put("lift pos 2", lift.getPos2());
            packet.put("lift power", lift.getPower());
            packet.put("thing", drive.getRawExternalHeadingDegrees());
            dashboard.sendTelemetryPacket(packet);
            lift.adjustLift();
            if(!drive.getIdle())
            {
                drive.update();
            }
        }
        while(IState==IntakeState.DONE&&!sensorController.depositNoCube())
        {
            servoControl.openDeposit();
        }
        leftIntake.setPower(0);
        lift.setPosition(0);
        servoControl.raiseAllIntakes();
        servoControl.startingPos();
        Trajectory park;

        park = drive.trajectoryBuilder(drive.getPoseEstimate())
                        .lineToSplineHeading(new Pose2d(60, drive.getPoseEstimate().getY(), Math.toRadians(0)))
                        .build();
        drive.followTrajectoryAsync(park);
        while(opModeIsActive()&&globalTimer.milliseconds()<29800&&(drive.isBusy()||lift.isBusy()))
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
            servoControl.flipOut();
            lift.setPosition(LIFT_EXTENDED);
            while(lift.isBusy())
            {
                //wait
            }
            servoControl.openDeposit();
            while(!sensorController.depositNoCube())
            {
                //wait
            }
        }
        //vision deposit code here
    }
}
