package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDriveCancelable;

public class TrajectoryGen
{
    SampleMecanumDriveCancelable drive;
    double cappedAccel;
    public TrajectoryGen(SampleMecanumDriveCancelable drive, double accel)
    {
        this.drive=drive;
        cappedAccel=accel;
    }

    public Trajectory preTrajectory(double distance)
    {
        Trajectory preTraj=drive.trajectoryBuilder(new Pose2d(0,0,0))
                .lineTo(new Vector2d(distance, 0))
                .build();
        return preTraj;
    }
    public Trajectory firstGoingTrajectory(double x1, double y1, double ang1, double x2, double y2, double ang2)
    {
        Trajectory firstGoingTraj = drive.trajectoryBuilder(new Pose2d())
                .splineTo(new Vector2d(x1, y1), Math.toRadians(ang1))
                .splineTo(new Vector2d(x2, y2), Math.toRadians(ang2), SampleMecanumDrive.getVelocityConstraint(cappedAccel,
                        DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                //.lineToSplineHeading(new Pose2d(73, -2.25, Math.toRadians(-10)))
                .build();
        return firstGoingTraj;
    }
    public Trajectory goingTrajectory(double x, double y, double ang)
        {
            Trajectory goingTraj = drive.trajectoryBuilder(drive.getPoseEstimate())
                    .lineToSplineHeading(new Pose2d(x, y, ang), SampleMecanumDrive.getVelocityConstraint(cappedAccel,
                            DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                    .build();
            return goingTraj;
        }
    public Trajectory returningTrajectory(double x, double y, double ang)
    {
        Trajectory returningTraj = drive.trajectoryBuilder(drive.getPoseEstimate())
                .lineToSplineHeading(new Pose2d(x, y, Math.toRadians(ang)))
                .build();
        return returningTraj;
    }
    public Trajectory realReturnTrajectory(double x1, double y1, double velocity)
    {
        Trajectory realReturn=drive.trajectoryBuilder(drive.getPoseEstimate(), true)
                .splineTo(new Vector2d(x1, y1), Math.toRadians(180))
                .addDisplacementMarker(()->
                {
                    //align into wall and reset pose estimate for y and heading
                    drive.setPoseEstimate(new Pose2d(drive.getPoseEstimate().getX(), 0, 0));
                })
                .splineTo(new Vector2d(0,0), Math.toRadians(180), SampleMecanumDrive.getVelocityConstraint(velocity,
                        DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
        return realReturn;
    }

}
