package org.firstinspires.ftc.teamcode;

import android.os.SystemClock;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDriveCancelable;

@Config
@Autonomous
public class RRTest extends LinearOpMode
{
    SampleMecanumDriveCancelable drive;
    SensorController sensorController;
    DcMotor leftIntake;
    ServoControl servoControl;
    public static double vel=40;
    public static double accel=30;
    public static double distance=80;
    double tempTarget=1000000000;
    boolean pain;
    int i=0;
    FtcDashboard dashboard;
    TelemetryPacket packet;
    double intakeSpeed;

    ColorRangeSensor intakeRange, driveLeft, driveRight, depositSensor;

    @Override
    public void runOpMode() throws InterruptedException {

        leftIntake = hardwareMap.dcMotor.get("leftIntake");
        driveLeft=hardwareMap.get(ColorRangeSensor.class, "driveSensor1");
        driveRight=hardwareMap.get(ColorRangeSensor.class, "driveSensor2");
        intakeSpeed=1;
        dashboard=FtcDashboard.getInstance();
        packet=new TelemetryPacket();
        drive=new SampleMecanumDriveCancelable(hardwareMap);
        sensorController=new SensorController(hardwareMap, SensorController.Side.LEFT);
        servoControl=new ServoControl(hardwareMap, ServoControl.Side.LEFT);
        drive.setPoseEstimate(new Pose2d(0, 0, 0));
        servoControl.lowerIntakes();
        servoControl.returnArm();
        waitForStart();
        Trajectory trajectory=drive.trajectoryBuilder(drive.getPoseEstimate())
                .lineToConstantHeading(new Vector2d(distance, 0), SampleMecanumDriveCancelable.getVelocityConstraint(vel,
                        DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDriveCancelable.getAccelerationConstraint(accel))
                .build();

        pain=false;
        i=0;
        //drive.followTrajectoryAsync(trajectory);
        double tempTarget=SystemClock.uptimeMillis()+5000;
        while(SystemClock.uptimeMillis()<tempTarget) {
            drive.setMotorPowers(0.2, 0.2, 0.2, 0.2);

        }
    }
}
