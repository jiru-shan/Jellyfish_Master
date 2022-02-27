package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp
public class SensorTesting extends LinearOpMode
{
    FtcDashboard dashboard;
    TelemetryPacket packet;
    SensorController sensorController;
    ElapsedTime loopTime;

    @Override
    public void runOpMode() throws InterruptedException
    {
        loopTime=new ElapsedTime();
        dashboard=FtcDashboard.getInstance();
        packet=new TelemetryPacket();
        sensorController=new SensorController(hardwareMap, SensorController.Side.LEFT);

        waitForStart();
        sensorController.closeIntakeSensor();
        while(opModeIsActive())
        {
            loopTime.reset();
            double[] pain=sensorController.getData();
            packet.put("intake 1 ", pain[0]);
            packet.put("deposit ", pain[1]);
            packet.put("drive left ", pain[2]);
            packet.put("drive right ", pain[3]);
            //packet.put("white line: ", sensorController.onColor());
            packet.put("loop time", loopTime.milliseconds());
            dashboard.sendTelemetryPacket(packet);
        }
    }
}
