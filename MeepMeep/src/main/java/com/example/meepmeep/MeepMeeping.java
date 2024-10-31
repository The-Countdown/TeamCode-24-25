package com.example.meepmeep;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeeping {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

//        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(29.34, 46.08, 270))
//                .splineToSplineHeading(new Pose2d(57.59, 39.97, Math.toRadians(270.00)), Math.toRadians(0.00))
//                .setReversed(true)
//                .splineToSplineHeading(new Pose2d(57.88, 56.86, Math.toRadians(45.00)), Math.toRadians(45.00))
//                .waitSeconds(1)
//                .setReversed(true)
//                .splineToSplineHeading(new Pose2d(24.97, 0, Math.toRadians(180.00)), Math.toRadians(180.00))
//                .waitSeconds(2)
//                .setReversed(true)
//                .splineToSplineHeading(new Pose2d(57.88, 56.86, Math.toRadians(45.00)), Math.toRadians(45.00))
//                .waitSeconds(1)
//                .setReversed(true)
//                .splineToSplineHeading(new Pose2d(24.97, 0, Math.toRadians(180.00)), Math.toRadians(180.00))
//                .waitSeconds(2)
//                .setReversed(true)
//                .splineToSplineHeading(new Pose2d(57.88, 56.86, Math.toRadians(45.00)), Math.toRadians(45.00))
//                .build());
        myBot.runAction(myBot.getDrive().actionBuilder (new Pose2d(12, 62, Math.toRadians(270)))
                .splineTo(new Vector2d(12.00, 34.00), Math.toRadians(270.00))
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(26.56, 52.41), Math.toRadians(270.00))
                .splineToSplineHeading(new Pose2d(35.85, 26.15, Math.toRadians(0.00)), Math.toRadians(260.00))
                .waitSeconds(0.25)
                .splineToConstantHeading(new Vector2d(40, 26.15), Math.toRadians(0.00))
                .waitSeconds(0.5)
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(57.88, 56.86, Math.toRadians(225)), Math.toRadians(45))
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(45, 26.15, Math.toRadians(0.00)), Math.toRadians(0.00))
                .waitSeconds(0.25)
                .splineToConstantHeading(new Vector2d(50, 26.15), Math.toRadians(0.00))
                .waitSeconds(0.5)
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(57.88, 56.86, Math.toRadians(225)), Math.toRadians(45))
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(55, 26.15, Math.toRadians(0.00)), Math.toRadians(0.00))
                .waitSeconds(0.25)
                .splineToConstantHeading(new Vector2d(60, 26.15), Math.toRadians(0.00))
                .waitSeconds(0.5)
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(57.88, 56.86, Math.toRadians(225)), Math.toRadians(45))
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}