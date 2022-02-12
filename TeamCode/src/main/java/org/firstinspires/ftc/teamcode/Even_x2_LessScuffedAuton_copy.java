package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
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

public class Even_x2_LessScuffedAuton_copy extends LinearOpMode
{
    //enum definitions
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

    //static variables for positions

    //changing variables that are used for stuff
    int cubePos;

    //instances of enums
    GrabbingState GState;
    ReturningState RState;
    IntakeState IState;

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
    public void runOpMode()
    {
        leftIntake = hardwareMap.dcMotor.get("leftIntake");
        rightIntake = hardwareMap.dcMotor.get("rightIntake");

        drive=new SampleMecanumDriveCancelable(hardwareMap);
        lift=new LiftAsync(hardwareMap, 0);
        sensorController=new SensorController(hardwareMap);
        servoControl=new ServoControl(hardwareMap, ServoControl.Side.LEFT);
        trajGen=new TrajectoryGen(drive, 40);

        globalTimer=new ElapsedTime();

        Pose2d startPose=new Pose2d(0,0,0);
        drive.setPoseEstimate(startPose);

        webcamInit();

        //initialize servos to correct starting position

        waitForStart();

        //globalTimer.reset();

        cubePos= pipeline.getAnalysis();

        drive.followTrajectory(trajGen.preTrajectory(-19.25));
        drive.setPoseEstimate(new Pose2d(0, 0, 0));

        visionDeposit(cubePos);

        //run cycles 6 times
        for(int i=0; i<6; i++)
        {
            drive.setPoseEstimate(new Pose2d(0,0,0));
            servoControl.lowerIntakes();
            //change some other positions

            



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
