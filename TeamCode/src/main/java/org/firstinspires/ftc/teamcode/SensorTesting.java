package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.vision.VisionPipeline_BLUE;
import org.firstinspires.ftc.teamcode.vision.VisionPipeline_BLUE_SCUFF;
import org.firstinspires.ftc.teamcode.vision.VisionPipeline_RED;
import org.firstinspires.ftc.teamcode.vision.VisionPipeline_RED_SCUFF;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@TeleOp
public class SensorTesting extends LinearOpMode
{
    FtcDashboard dashboard;
    TelemetryPacket packet;
    SensorController sensorController;
    ElapsedTime loopTime;

    int cubePos;

    OpenCvWebcam camera;
    VisionPipeline_BLUE_SCUFF pipeline;


    @Override
    public void runOpMode() throws InterruptedException
    {
        loopTime=new ElapsedTime();
        dashboard=FtcDashboard.getInstance();
        packet=new TelemetryPacket();
        sensorController=new SensorController(hardwareMap, SensorController.Side.LEFT);

        webcamInit();
        waitForStart();
        dashboard.startCameraStream(camera, 30);
        sensorController.closeIntakeSensor();

        while(opModeIsActive())
        {
            loopTime.reset();
            double[] pain=sensorController.getData();
            packet.put("intake 1 ", pain[0]);
            packet.put("deposit ", pain[1]);
            packet.put("drive left ", pain[2]);
            packet.put("drive right ", pain[3]);
            packet.put("drive left raw ", pain[4]);
            packet.put("drive right raw ", pain[5]);
            packet.put("intake boolean", sensorController.test());
            //packet.put("white line: ", sensorController.onColor());
            packet.put("loop time", loopTime.milliseconds());
            dashboard.sendTelemetryPacket(packet);
            packet.put("vision zone ", pipeline.getAnalysis());
        }
    }

    public void webcamInit()
    {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam"), cameraMonitorViewId);
        pipeline=new VisionPipeline_BLUE_SCUFF(telemetry);
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
}
