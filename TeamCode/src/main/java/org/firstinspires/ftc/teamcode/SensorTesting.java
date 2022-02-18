package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class SensorTesting extends LinearOpMode
{
    FtcDashboard dashboard;
    TelemetryPacket packet;
    SensorController sensorController;

    @Override
    public void runOpMode() throws InterruptedException
    {
        dashboard=FtcDashboard.getInstance();
        packet=new TelemetryPacket();
        sensorController=new SensorController(hardwareMap);

        waitForStart();
        while(opModeIsActive())
        {
            double[] pain=sensorController.getData();
            packet.put("intake 1 ", pain[0]);
            packet.put("intake 2 ", pain[1]);
            packet.put("deposit ", pain[2]);
            packet.put("white line: ", sensorController.onColor());

            dashboard.sendTelemetryPacket(packet);
        }
    }
}
