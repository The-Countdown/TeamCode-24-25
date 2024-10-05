package com.example.meepmeep;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeeping {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(600);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(29.34, 46.08, 270))
                .splineToSplineHeading(new Pose2d(57.59, 39.97, Math.toRadians(270.00)), Math.toRadians(0.00))
                .setReversed(true)
                .splineToSplineHeading(new Pose2d(57.88, 56.86, Math.toRadians(45.00)), Math.toRadians(45.00))
                .waitSeconds(1)
                .setReversed(true)
                .splineToSplineHeading(new Pose2d(24.97, 0, Math.toRadians(180.00)), Math.toRadians(180.00))
                .waitSeconds(2)
                .setReversed(true)
                .splineToSplineHeading(new Pose2d(57.88, 56.86, Math.toRadians(45.00)), Math.toRadians(45.00))
                .waitSeconds(1)
                .setReversed(true)
                .splineToSplineHeading(new Pose2d(24.97, 0, Math.toRadians(180.00)), Math.toRadians(180.00))
                .waitSeconds(2)
                .setReversed(true)
                .splineToSplineHeading(new Pose2d(57.88, 56.86, Math.toRadians(45.00)), Math.toRadians(45.00))
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}