package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.path.Path;
import com.acmerobotics.roadrunner.path.PathSegment;
import com.acmerobotics.roadrunner.path.QuinticSpline;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryGenerator;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.SampleMecanumDrive;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting
{
    public static double TURN_ANGLE=-30;
    public static void main(String[] args) {

        MeepMeep meepMeep = new MeepMeep(800);

        QuinticSpline spline=new QuinticSpline(
                new QuinticSpline.Knot(0,0,-100,10),
                new QuinticSpline.Knot(-65, -2.25, -100, 10), 100, 100, 100
        );
        PathSegment pathSegment= new PathSegment(spline);
        Path path=new Path(pathSegment);
        Trajectory traj= TrajectoryGenerator.INSTANCE.generateTrajectory(path, SampleMecanumDrive.getVelocityConstraint(70,
                70, 20), SampleMecanumDrive.getAccelerationConstraint(20));
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(0, 0, 0))
                                //.setReversed(true)
                               // .strafeRight(15)
                                //.lineToConstantHeading(new Vector2d(0,2))
                                //.addTrajectory(traj)

                                //.splineTo(new Vector2d(73, -3.1), Math.toRadians(-7.8))

                                .splineTo(new Vector2d(49, 13), Math.toRadians(180))
                               // .forward(10)
                               // .splineTo(new Vector2d(30, -1), Math.toRadians(-5))
                                //.setReversed(true)
                                //.setReversed(false)
                                //.lineToSplineHeading(new Pose2d(0, 13, Math.toRadians(0)))
                                ///.lineToSplineHeading(new Pose2d(49, 13, Math.toRadians(0)))

                                //.splineTo(new Vector2d(0, 13), Math.toRadians(180))
                               //.lineToConstantHeading(new Vector2d(0, 0))
                                //.setReversed(true)
                                //.lineToConstantHeading(new Vector2d(-42, -1))
                               // .back(49)
                               // .splineTo(new Vector2d(0, 0), Math.toRadians(180))
                               // .lineToSplineHeading(new Pose2d(-50, 0, Math.toRadians(0)))
                                //.lineToSplineHeading(new Pose2d(60, -3.25, Math.toRadians(-10)))
                                //.setReversed(true)
                                //.lineToSplineHeading(new Pose2d(29, 6, Math.toRadians(0)))
                                //.lineToConstantHeading(new Vector2d(42, -1))
                                .build()
                );
        meepMeep.setBackground(MeepMeep.Background.FIELD_FREIGHTFRENZY_ADI_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}