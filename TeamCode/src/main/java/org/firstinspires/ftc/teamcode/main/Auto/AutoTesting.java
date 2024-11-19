package org.firstinspires.ftc.teamcode.main.Auto;

import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.main.Auto.RoadRunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.subsystems.actions.Wait;
import org.firstinspires.ftc.teamcode.subsystems.actions.intake.IntakeWait;
import org.firstinspires.ftc.teamcode.subsystems.actions.outtake.OuttakeCondense;
import org.firstinspires.ftc.teamcode.subsystems.actions.outtake.OuttakePreloadEsc;
import org.firstinspires.ftc.teamcode.subsystems.actions.outtake.OuttakeSpecimen;

@Autonomous
public class AutoTesting extends LinearOpMode {

    @Override
    public void runOpMode() {
        Robot robot = new Robot(this);
        MecanumDrive drive = new MecanumDrive(hardwareMap, robot.beginPose);
        drive.updatePoseEstimate();

        TrajectoryActionBuilder toSubmersibleFromStart = drive.actionBuilder(robot.beginPose)
                .strafeTo(new Vector2d(43, 0));

        TrajectoryActionBuilder toFirstSample = drive.actionBuilder(new Pose2d(43, 0, 0))
                .strafeTo(new Vector2d(32, 0))
                .strafeTo(new Vector2d(32, -55))
                .strafeTo(new Vector2d(75, -55))
                .strafeTo(new Vector2d(75, -71));

        TrajectoryActionBuilder toWallFromFirstSample = drive.actionBuilder(new Pose2d(75, -71, 0))
                .strafeTo(new Vector2d(10, -69));

        TrajectoryActionBuilder toSecondSample = drive.actionBuilder(new Pose2d(10, -69, 0))
                .strafeTo(new Vector2d(74, -69))
                .strafeTo(new Vector2d(72, -83));

        TrajectoryActionBuilder toWallFromSecondSample = drive.actionBuilder(new Pose2d(72, -83, 0))
                .strafeTo(new Vector2d(11, -83));

        TrajectoryActionBuilder toAwayFromWallAfterPush = drive.actionBuilder(new Pose2d(11, -83, 0))
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(24, -73, Math.toRadians(180)), Math.toRadians(0));

        TrajectoryActionBuilder toSpecimenFromAwayFromWall = drive.actionBuilder(new Pose2d(24, -73, Math.toRadians(180)))
                .strafeTo(new Vector2d(11, -73));

        TrajectoryActionBuilder toSubmersibleFromSpecimenFirst = drive.actionBuilder(new Pose2d(11, -73, Math.toRadians(180)))
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(37, -6, Math.toRadians(0)), Math.toRadians(0))
                .strafeTo(new Vector2d(43, -6));

        TrajectoryActionBuilder toSpecimenFromSubmersibleFirst = drive.actionBuilder(new Pose2d(43, -6, 0))
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(24, -73, Math.toRadians(180)), Math.toRadians(0))
                .waitSeconds(1)
                .strafeTo(new Vector2d(11, -73));

        TrajectoryActionBuilder toSubmersibleFromSpecimenSecond = drive.actionBuilder(new Pose2d(11, -73, Math.toRadians(180)))
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(37, 0, Math.toRadians(0)), Math.toRadians(0))
                .strafeTo(new Vector2d(43, 0));

        robot.intake.rest();
        robot.outtake.rest();

        waitForStart();

        Actions.runBlocking(new SequentialAction(
                toSubmersibleFromStart.build(),
                toFirstSample.build(),
                toWallFromFirstSample.build(),
                toSecondSample.build(),
                toWallFromSecondSample.build(),
                toAwayFromWallAfterPush.build(),
                toSpecimenFromAwayFromWall.build(),
                toSubmersibleFromSpecimenFirst.build(),
                toSpecimenFromSubmersibleFirst.build(),
                toSubmersibleFromSpecimenSecond.build()
        ));
    }
}
