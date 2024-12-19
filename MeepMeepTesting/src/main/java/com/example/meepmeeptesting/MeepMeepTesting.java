package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.rowlandhall.meepmeep.MeepMeep;
import org.rowlandhall.meepmeep.roadrunner.DefaultBotBuilder;
import org.rowlandhall.meepmeep.roadrunner.entity.RoadRunnerBotEntity;
import org.rowlandhall.meepmeep.roadrunner.trajectorysequence.TrajectorySequence;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(90, 90, Math.toRadians(322.28875976108804), Math.toRadians(322.28875976108804), 14)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(-0.07, 61.75, Math.toRadians(90.00)))
                        .lineTo(new Vector2d(58.96, 61.89))
                        .lineTo(new Vector2d(29.66, 62.04))
                        .lineTo(new Vector2d(30.25, 0.66))
                        .lineToSplineHeading(new Pose2d(13.84, 0.37, Math.toRadians(180.00)))
                        .build());



        meepMeep.setBackground(MeepMeep.Background.FIELD_INTOTHEDEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}