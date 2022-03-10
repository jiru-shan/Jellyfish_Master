package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDriveCancelable;

public class TrajectoryGen
{
    //maybe increase acceleration
    SampleMecanumDriveCancelable drive;
    double cappedVel;
    public TrajectoryGen(SampleMecanumDriveCancelable drive, double vel)
    {
        this.drive=drive;
        cappedVel =vel;
    }

    public Trajectory preTrajectory(double distance)
    {
        Trajectory preTraj=drive.trajectoryBuilder(new Pose2d(0,0,0))
                .lineTo(new Vector2d(distance, 0))
                .build();
        return preTraj;
    }
    public Trajectory firstGoingTrajectory(double x1, double y1, double ang1, double x2, double y2, double ang2, boolean reversed)
    {
        Trajectory firstGoingTraj = drive.trajectoryBuilder(new Pose2d(), reversed)
                .splineTo(new Vector2d(x1, y1), Math.toRadians(ang1))
                .splineTo(new Vector2d(x2, y2), Math.toRadians(ang2), SampleMecanumDriveCancelable.getVelocityConstraint(cappedVel,
                        DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDriveCancelable.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                //.lineToSplineHeading(new Pose2d(73, -2.25, Math.toRadians(-10)))
                .build();
        return firstGoingTraj;
    }
    public Trajectory goingTrajectory(double x, double y, double ang)
        {
            Trajectory goingTraj = drive.trajectoryBuilder(drive.getPoseEstimate())
                    .lineToSplineHeading(new Pose2d(x, y, ang), SampleMecanumDriveCancelable.getVelocityConstraint(cappedVel,
                            DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDriveCancelable.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
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

    public Trajectory realReturnTrajectory()
    {
        Trajectory realReturn=drive.trajectoryBuilder(drive.getPoseEstimate())
                .lineToSplineHeading(new Pose2d(0, 13, Math.toRadians(0)), SampleMecanumDriveCancelable.getVelocityConstraint(100,
                        DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDriveCancelable.getAccelerationConstraint(100))
                .build();
        return realReturn;
    }



}
