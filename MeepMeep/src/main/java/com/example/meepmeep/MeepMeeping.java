package com.example.meepmeep;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeeping {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(700);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder (new Pose2d(12, 62, Math.toRadians(270)))
                .splineTo(new Vector2d(12.00, 34.00), Math.toRadians(270.00))
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(26.56, 52.41), Math.toRadians(270.00))
                .splineToSplineHeading(new Pose2d(35.85, 26.15, Math.toRadians(0.00)), Math.toRadians(260.00))
                .waitSeconds(0.25)
                .splineToConstantHeading(new Vector2d(40, 26.15), Math.toRadians(0.00))
                .waitSeconds(0.5)
                .strafeToConstantHeading(new Vector2d (40, 35))
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(57.88, 56.86, Math.toRadians(225)), Math.toRadians(45))
                .setReversed(false)
                .splineToLinearHeading(new Pose2d(45, 26.15, Math.toRadians(0.00)), Math.toRadians(0))
                .waitSeconds(0.25)
                .splineToConstantHeading(new Vector2d(50, 26.15), Math.toRadians(0.00))
                .waitSeconds(0.5)
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(57.88, 56.86, Math.toRadians(225)), Math.toRadians(45))
                .setReversed(false)
                .splineToLinearHeading(new Pose2d(55, 26.15, Math.toRadians(0.00)), Math.toRadians(0))
                .waitSeconds(0.25)
                .splineToConstantHeading(new Vector2d(60, 26.15), Math.toRadians(0.00))
                .waitSeconds(0.5)
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(57.88, 56.86, Math.toRadians(225)), Math.toRadians(45))
                .build());

        RoadRunnerBotEntity mySecondBot = new DefaultBotBuilder(meepMeep)
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        mySecondBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-12, -62, Math.toRadians(90)))
                .splineTo(new Vector2d(-12.00, -34.00), Math.toRadians(90.00))
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(-26.56, -52.41), Math.toRadians(90.00))
                .splineToSplineHeading(new Pose2d(-35.85, -26.15, Math.toRadians(180.00)), Math.toRadians(100.00))
                .waitSeconds(0.25)
                .splineToConstantHeading(new Vector2d(-40, -26.15), Math.toRadians(180.00))
                .waitSeconds(0.5)
                .strafeToConstantHeading(new Vector2d (-40, -35))
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(-57.88, -56.86, Math.toRadians(45)), Math.toRadians(225))
                .setReversed(false)
                .splineToLinearHeading(new Pose2d(-45, -26.15, Math.toRadians(180.00)), Math.toRadians(180.00))
                .waitSeconds(0.25)
                .splineToConstantHeading(new Vector2d(-50, -26.15), Math.toRadians(180.00))
                .waitSeconds(0.5)
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(-57.88, -56.86, Math.toRadians(45)), Math.toRadians(225))
                .setReversed(false)
                .splineToLinearHeading(new Pose2d(-55, -26.15, Math.toRadians(180.00)), Math.toRadians(180.00))
                .waitSeconds(0.25)
                .splineToConstantHeading(new Vector2d(-60, -26.15), Math.toRadians(180.00))
                .waitSeconds(0.5)
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(-57.88, -56.86, Math.toRadians(45)), Math.toRadians(225))
                .build());

        RoadRunnerBotEntity myThirdBot = new DefaultBotBuilder(meepMeep)
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        myThirdBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(0, 0, Math.toRadians(270)))
                .splineToConstantHeading(new Vector2d(0, -18), Math.toRadians(270.00))
                .splineToSplineHeading(new Pose2d(28, -8, Math.toRadians(45)), Math.toRadians(0))
                .build());

                meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .addEntity(mySecondBot)
                .addEntity(myThirdBot)
                .start();
    }
}