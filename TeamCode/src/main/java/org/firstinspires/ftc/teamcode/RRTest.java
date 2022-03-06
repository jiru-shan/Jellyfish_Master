package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDriveCancelable;

@Config
@Autonomous
public class RRTest extends LinearOpMode
{
    SampleMecanumDriveCancelable drive;
    SensorController sensorController;
    public static double vel=40;
    public static double accel=30;
    public static double distance=80;
    boolean pain;
    FtcDashboard dashboard;
    TelemetryPacket packet;

    ColorRangeSensor intakeRange, driveLeft, driveRight, depositSensor;

    @Override
    public void runOpMode() throws InterruptedException {
        driveLeft=hardwareMap.get(ColorRangeSensor.class, "driveSensor1");
        driveRight=hardwareMap.get(ColorRangeSensor.class, "driveSensor2");

        dashboard=FtcDashboard.getInstance();
        packet=new TelemetryPacket();
        drive=new SampleMecanumDriveCancelable(hardwareMap);
        sensorController=new SensorController(hardwareMap, SensorController.Side.LEFT);
        drive.setPoseEstimate(new Pose2d(0, 0, 0));

        waitForStart();
        Trajectory trajectory=drive.trajectoryBuilder(drive.getPoseEstimate())
                .lineToConstantHeading(new Vector2d(distance, 0), SampleMecanumDriveCancelable.getVelocityConstraint(vel,
                        DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDriveCancelable.getAccelerationConstraint(accel))
                .build();

        drive.followTrajectoryAsync(trajectory);
        pain=false;
        while(opModeIsActive()) {
            if(!pain)
            {
                drive.update();
            }
            else
                {
                    drive.setDrivePower(new Pose2d(5, 0, 0));
                }
            if (driveLeft.getRawLightDetected() > 850 || driveRight.getRawLightDetected() > 850) {
                pain = true;
            }
            packet.put("help", pain);
            dashboard.sendTelemetryPacket(packet);
        }
    }
}
