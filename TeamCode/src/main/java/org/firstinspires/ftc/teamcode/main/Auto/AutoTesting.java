package org.firstinspires.ftc.teamcode.main.Auto;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.main.Auto.RoadRunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

@Autonomous
public class AutoTesting extends LinearOpMode {

    @Override
    public void runOpMode() {
        Robot robot = new Robot(this);

        TrajectoryActionBuilder toSubmersibleFromStart = robot.roadRunner.actionBuilder(robot.beginPose)
                .strafeTo(new Vector2d(43, 0));

        TrajectoryActionBuilder toWallFromSecondSample = robot.roadRunner.actionBuilder(new Pose2d(43, 0, 0))
//                .strafeTo(new Vector2d(32, 0))
                .strafeTo(new Vector2d(32, -55))
                .strafeTo(new Vector2d(75, -55))
//                .strafeToLinearHeading(new Vector2d(18, -15), 0)
//                .strafeToLinearHeading(new Vector2d(58, -55), 0)
                .splineToConstantHeading(new Vector2d(75, -71), 0)
                .splineToConstantHeading(new Vector2d(10, -69), 0)
                .splineToConstantHeading(new Vector2d(75, -69), 0)
                .splineToConstantHeading(new Vector2d(75, -83), 0)
                .splineToConstantHeading(new Vector2d(11, -83), 0);

        TrajectoryActionBuilder toAwayFromWallAfterPush = robot.roadRunner.actionBuilder(new Pose2d(11, -83, 0))
                .setReversed(true)
                .strafeToLinearHeading(new Vector2d(24, -73), Math.toRadians(180));

        TrajectoryActionBuilder toSpecimenFromAwayFromWall = robot.roadRunner.actionBuilder(new Pose2d(24, -73, Math.toRadians(180)))
                .strafeTo(new Vector2d(11, -73));

        TrajectoryActionBuilder toSubmersibleFromSpecimenFirst = robot.roadRunner.actionBuilder(new Pose2d(11, -73, Math.toRadians(180)))
                .setReversed(true)
                .strafeToLinearHeading(new Vector2d(37, -6), Math.toRadians(0))
                .strafeTo(new Vector2d(43, -6));

        TrajectoryActionBuilder toSpecimenFromSubmersibleFirst = robot.roadRunner.actionBuilder(new Pose2d(43, -6, 0))
                .setReversed(true)
                .strafeToLinearHeading(new Vector2d(24, -73), Math.toRadians(180))
                .waitSeconds(1)
                .strafeTo(new Vector2d(11, -73));

        TrajectoryActionBuilder toSubmersibleFromSpecimenSecond = robot.roadRunner.actionBuilder(new Pose2d(11, -73, Math.toRadians(180)))
                .setReversed(true)
                .strafeToLinearHeading(new Vector2d(37, 0), Math.toRadians(0))
                .strafeTo(new Vector2d(43, 0));

        robot.intake.rest();
        robot.outtake.rest();

        waitForStart();

        Actions.runBlocking(new SequentialAction(
                toSubmersibleFromStart.build(),
                toWallFromSecondSample.build(),
                toAwayFromWallAfterPush.build(),
                toSpecimenFromAwayFromWall.build(),
                toSubmersibleFromSpecimenFirst.build(),
                toSpecimenFromSubmersibleFirst.build(),
                toSubmersibleFromSpecimenSecond.build()
        ));
    }
}
