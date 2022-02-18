package org.firstinspires.ftc.teamcode.past_code;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.BNO055IMUImpl;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class ImuTesting extends LinearOpMode
{
    BNO055IMU imu;
    FtcDashboard dashboard;
    TelemetryPacket packet;


    @Override
    public void runOpMode() throws InterruptedException {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        //parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hardwareMap.get(BNO055IMUImpl.class, "imu");
        imu.initialize(parameters);

        dashboard=FtcDashboard.getInstance();
        packet=new TelemetryPacket();
        imu=hardwareMap.get(BNO055IMU.class, "imu");

        waitForStart();
        while(opModeIsActive())
        {
            packet.put("X", getX());
            packet.put("Y", getY());
            packet.put("Z", getZ());

            dashboard.sendTelemetryPacket(packet);
        }
    }

    public double getX()
    {
        return imu.getAngularOrientation().firstAngle;

       /* return imu.getAngularOrientation().firstAngle*Math.sin(Math.toRadians(40))
                +imu.getAngularOrientation().secondAngle*Math.cos(Math.toRadians(40));*/
    }
    public double getY() {
        return imu.getAngularOrientation().firstAngle;

        /*return imu.getAngularOrientation().firstAngle * (Math.cos(Math.toRadians(40)))
        +imu.getAngularOrientation().secondAngle*Math.sin(Math.toRadians(40));*/
    }
    public double getZ()
    {
        return imu.getAngularOrientation().thirdAngle;
    }
}
