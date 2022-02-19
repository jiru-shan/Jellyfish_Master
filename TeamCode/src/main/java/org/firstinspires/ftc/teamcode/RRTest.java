package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDriveCancelable;

@Autonomous
public class RRTest extends LinearOpMode
{
    SampleMecanumDriveCancelable drive;
    SensorController sensorController;

    @Override
    public void runOpMode() throws InterruptedException {
        drive=new SampleMecanumDriveCancelable(hardwareMap);
        sensorController=new SensorController(hardwareMap);
        drive.setPoseEstimate(new Pose2d(30, 0, 0));

        waitForStart();
        Trajectory trajectory=drive.trajectoryBuilder(drive.getPoseEstimate())
                .lineToConstantHeading(new Vector2d(0, 0))
                .build();

        drive.followTrajectoryAsync(trajectory);
        while(opModeIsActive())
        {
            drive.update();
            if(sensorController.onColor())
            {
                drive.setPoseEstimate(new Pose2d(50, 0, 0));
                telemetry.addData("pain", "is real");
                telemetry.update();
            }
        }
    }
}
