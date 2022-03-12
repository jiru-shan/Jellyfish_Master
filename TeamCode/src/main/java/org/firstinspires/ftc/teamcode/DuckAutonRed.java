package org.firstinspires.ftc.teamcode;

import android.os.SystemClock;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDriveCancelable;
import org.firstinspires.ftc.teamcode.vision.VisionPipeline_BLUE;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous
public class DuckAutonRed extends LinearOpMode
{
    CRServo carousel, carousel2;

    int cubePos=1;

    OpenCvWebcam camera;
    VisionPipeline_BLUE pipeline;

    SampleMecanumDriveCancelable drive;

    @Override
    public void runOpMode() throws InterruptedException
    {
        carousel=hardwareMap.get(CRServo.class, "c_Left");
        carousel2=hardwareMap.get(CRServo.class, "c_Right");


        drive=new SampleMecanumDriveCancelable(hardwareMap);
        Pose2d startPose= new Pose2d(-34, 62, 0);
        drive.setPoseEstimate(startPose);
        webcamInit();

        //read vision place
        waitForStart();
        cubePos= pipeline.getAnalysis();
        //camera.stopStreaming();
        //camera.closeCameraDevice();

        //move from reading vision to deposit place
        Trajectory trajectory=drive.trajectoryBuilder(drive.getPoseEstimate())
                .back(22, SampleMecanumDriveCancelable.getVelocityConstraint(50,
                        DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDriveCancelable.getAccelerationConstraint(30))
                .build();
        drive.followTrajectory(trajectory);

        //deposit
        visionDeposit(cubePos);

        //move to carousel
        trajectory =drive.trajectoryBuilder(drive.getPoseEstimate())
                .forward(37, SampleMecanumDriveCancelable.getVelocityConstraint(50,
                        DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDriveCancelable.getAccelerationConstraint(30))
                .build();
        drive.followTrajectory(trajectory);

        trajectory =drive.trajectoryBuilder(drive.getPoseEstimate())
                .strafeRight(16, SampleMecanumDriveCancelable.getVelocityConstraint(50,
                        DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDriveCancelable.getAccelerationConstraint(30))
                .build();
        drive.followTrajectory(trajectory);

        trajectory =drive.trajectoryBuilder(drive.getPoseEstimate())
                .forward(13, SampleMecanumDriveCancelable.getVelocityConstraint(50,
                        DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDriveCancelable.getAccelerationConstraint(30))
                .build();
        drive.followTrajectory(trajectory);

        trajectory =drive.trajectoryBuilder(drive.getPoseEstimate())
                .strafeLeft(11, SampleMecanumDriveCancelable.getVelocityConstraint(20,
                        DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDriveCancelable.getAccelerationConstraint(10))
                .build();
        drive.followTrajectory(trajectory);

        //turn carousel
        double tempTarget= SystemClock.uptimeMillis()+3500;
        while(SystemClock.uptimeMillis()<tempTarget)
        {
            carousel.setPower(0.25);
            carousel2.setPower(0.25);
        }

        carousel.setPower(0);
        carousel2.setPower(0);
        //park
        trajectory =drive.trajectoryBuilder(drive.getPoseEstimate())
                .strafeRight(26, SampleMecanumDriveCancelable.getVelocityConstraint(50,
                        DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDriveCancelable.getAccelerationConstraint(30))
                .build();
        drive.followTrajectory(trajectory);

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
    public void visionDeposit(int pos)
    {
        if(pos==1){}
        else if(pos==2)
        {

        }
        else if(pos==3)
        {

        }
    }
}
