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
                //BLUE BASKET STARTS HERE
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder
                                        (new Pose2d(20, 65, Math.toRadians(-90)))
                                // This is for Blue (Basket Side)


                                //Path for Blue Basket Drop
                                .lineToSplineHeading(new Pose2d(37, 37, Math.toRadians(-45)))
                                .lineToSplineHeading(new Pose2d(53, 57, Math.toRadians(45)))
                                .lineToSplineHeading(new Pose2d(47, 37, Math.toRadians(-45)))
                                .lineToSplineHeading(new Pose2d(53, 57, Math.toRadians(45)))
                                .lineToSplineHeading(new Pose2d(59, 37, Math.toRadians(-45)))
                                .lineToSplineHeading(new Pose2d(53, 57, Math.toRadians(45)))
                                .lineToSplineHeading(new Pose2d(25, 25, Math.toRadians(0)))

                                /*
                                This is Blue Basket Drop, but without Spline

                                .forward(2)
                                .turn(Math.toRadians(45))
                                .forward(35)
                                .turn(Math.toRadians(-45))
                                .forward(-25)
                                .turn(Math.toRadians(90))
                                .turn(Math.toRadians(-70))
                                .forward(30)
                                .forward(-30)
                                .turn(Math.toRadians(70))
                                .turn(Math.toRadians(-60))
                                .forward(30)
                                .forward(-30)
                                .turn(Math.toRadians(60))
                                .turn(Math.toRadians(-110))
                                .forward(50)
                                */
                                //BLUE BASKET ENDS HERE

                                .build());



        meepMeep.setBackground(MeepMeep.Background.FIELD_INTOTHEDEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}