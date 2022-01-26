package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting
{
    public static double TURN_ANGLE=-30;
    public static void main(String[] args) {

        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(0, 0, 0))
                                //.setReversed(true)
                                //.lineToConstantHeading(new Vector2d(-42, -1))
                                //.splineTo(new Vector2d(-42, -1), Math.toRadians(5))
                                .lineToSplineHeading(new Pose2d(51, -3.25,Math.toRadians(-10)))
                                //.lineToSplineHeading(new Pose2d(60, -3.25, Math.toRadians(-10)))
                                //.setReversed(true)
                                .lineToSplineHeading(new Pose2d(29, 6, Math.toRadians(0)))
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