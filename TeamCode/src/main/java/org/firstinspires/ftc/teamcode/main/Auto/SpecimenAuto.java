package org.firstinspires.ftc.teamcode.main.Auto;

import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.subsystems.TeleOpPoseUpdater;
import org.firstinspires.ftc.teamcode.subsystems.actions.Wait;
import org.firstinspires.ftc.teamcode.subsystems.actions.intake.IntakeEsc;
import org.firstinspires.ftc.teamcode.subsystems.actions.outtake.OuttakeClip;
import org.firstinspires.ftc.teamcode.subsystems.actions.outtake.OuttakePreloadEsc;
import org.firstinspires.ftc.teamcode.subsystems.actions.outtake.OuttakeSpecimen;

@Autonomous(name = "Specimen Auto")
public class SpecimenAuto extends LinearOpMode {
    @Override
    public void runOpMode() {
        Robot robot = new Robot(this, new Pose2d(0, 0, 0));
        robot.intake.rest();
        robot.outtake.rest();
        robot.limeLight.limeLightInit(Robot.color, 100);

        TrajectoryActionBuilder toFirstClip = robot.roadRunner.actionBuilder(robot.beginPose)
                .afterTime(0.01, new OuttakePreloadEsc())
                .afterTime(0.75, new InstantAction(() -> Robot.rb.depositSlide.move(250)))
                .strafeToLinearHeading(new Vector2d(-47, -6), Math.toRadians(0));

        TrajectoryActionBuilder toFirstGrab = toFirstClip.fresh()
                .strafeToLinearHeading(new Vector2d(-34, 36), Math.toRadians(130));

        TrajectoryActionBuilder toPlace = robot.roadRunner.actionBuilder(new Pose2d(-34, 36, Math.toRadians(130)))
                .strafeToLinearHeading(new Vector2d(-15, 42), Math.toRadians(40));

        TrajectoryActionBuilder toSecondGrab = robot.roadRunner.actionBuilder(new Pose2d(-15, 42, Math.toRadians(40)))
                .strafeToLinearHeading(new Vector2d(-37, 47), Math.toRadians(121));

        TrajectoryActionBuilder toPlace2 = robot.roadRunner.actionBuilder(new Pose2d(-37, 47, Math.toRadians(121)))
                .strafeToLinearHeading(new Vector2d(-15, 42), Math.toRadians(40));

        TrajectoryActionBuilder toWallPrep = robot.roadRunner.actionBuilder(new Pose2d(-15, 42, Math.toRadians(40)))
                .strafeToLinearHeading(new Vector2d(-19, 50), Math.toRadians(0));

        TrajectoryActionBuilder toWallPrepClip = toFirstClip.fresh()
                .strafeToLinearHeading(new Vector2d(-19, 50), Math.toRadians(0));

        TrajectoryActionBuilder toWall = robot.roadRunner.actionBuilder(new Pose2d(-19, 50, Math.toRadians(0)))
                .strafeTo(new Vector2d(-4, 50));

        TrajectoryActionBuilder toWallFirst = robot.roadRunner.actionBuilder(new Pose2d(-19, 50, Math.toRadians(0)))
                .strafeTo(new Vector2d(-5, 50));

        TrajectoryActionBuilder toSecondClip = robot.roadRunner.actionBuilder(new Pose2d(-5, 58, Math.toRadians(0)))
                .strafeToLinearHeading(new Vector2d(-47, -11), Math.toRadians(0));

        TrajectoryActionBuilder toThirdClip = robot.roadRunner.actionBuilder(new Pose2d(-4, 58, Math.toRadians(0)))
                .strafeToLinearHeading(new Vector2d(-47, -16), Math.toRadians(0));

        TrajectoryActionBuilder toWallPrepBackup = robot.roadRunner.actionBuilder(new Pose2d(-47, -14, Math.toRadians(0)))
                .strafeToLinearHeading(new Vector2d(-30, -14), Math.toRadians(0))
                .strafeToLinearHeading(new Vector2d(-19, 50), Math.toRadians(0));

        TrajectoryActionBuilder toFourthClip = robot.roadRunner.actionBuilder(new Pose2d(-4, 58, Math.toRadians(0)))
                .strafeToLinearHeading(new Vector2d(-47, -21), Math.toRadians(0));

        telemetry.addData("Color", Robot.color);
        telemetry.update();

        waitForStart();

        Robot.isAutoReversed = true;

        TeleOpPoseUpdater teleOpPoseUpdater = new TeleOpPoseUpdater();
        Thread teleOpPoseUpdaterThread = new Thread(teleOpPoseUpdater);
        teleOpPoseUpdaterThread.start();

        Actions.runBlocking(new SequentialAction(
                toFirstClip.build(),
                new IntakeEsc(),
                new OuttakeClip(),
                new InstantAction(() -> robot.intake.down()),
                new InstantAction(() -> robot.intake.arm.up()),
                toFirstGrab.build(),
                new OuttakeSpecimen(),
                new InstantAction(() -> robot.outtake.arm.upLift()),
                new InstantAction(() -> robot.intakeSlide.moveTo(520)),
                new Wait(300),
                new InstantAction(() -> robot.limeLight.pickUp()),
                new InstantAction(() -> robot.intakeSlide.moveTo(664)),
                toPlace.build(),
                new InstantAction(() -> robot.intake.hand.open()),
                new Wait(300),
                toSecondGrab.build(),
                new InstantAction(() -> robot.intakeSlide.moveTo(800)),
                new Wait(300),
                new InstantAction(() -> robot.limeLight.pickUp()),
                new InstantAction(() -> robot.intakeSlide.moveTo(664)),
                toPlace2.build(),
                new InstantAction(() -> robot.intake.hand.open()),
                new Wait(300),
                new InstantAction(() -> robot.intake.rest()),
                new InstantAction(() -> robot.depositSlide.move(0)),
                new InstantAction(() -> robot.intakeSlide.moveTo(0)),
                new InstantAction(() -> robot.outtake.arm.upClip()),
                new InstantAction(() -> robot.outtake.wrist.horizontal()),
                new InstantAction(() -> robot.outtake.hand.open()),
                toWallPrep.build(),
                toWallFirst.build(),
                new InstantAction(() -> robot.outtake.hand.close()),
                new Wait(300),
                new InstantAction(() -> robot.depositSlide.move(250)),
                new InstantAction(() -> robot.outtake.arm.upLift()),
                new InstantAction(() -> robot.outtake.wrist.horizontalFlip()),
                toSecondClip.build(),
                new OuttakeClip(),
                new Wait(100),
                new OuttakeSpecimen(),
                toWallPrepClip.build(),
                toWall.build(),
                new InstantAction(() -> robot.outtake.hand.close()),
                new Wait(300),
                new InstantAction(() -> robot.depositSlide.move(250)),
                new InstantAction(() -> robot.outtake.arm.upLift()),
                new InstantAction(() -> robot.outtake.wrist.horizontalFlip()),
                toThirdClip.build(),
                new OuttakeClip(),
                new Wait(100),
                new OuttakeSpecimen(),
                toWallPrepBackup.build(),
                toWall.build(),
                new InstantAction(() -> robot.outtake.hand.close()),
                new Wait(300),
                new InstantAction(() -> robot.depositSlide.move(250)),
                new InstantAction(() -> robot.outtake.arm.upLift()),
                new InstantAction(() -> robot.outtake.wrist.horizontalFlip()),
                toFourthClip.build(),
                new OuttakeClip(),
                new Wait(100),
                new OuttakeSpecimen(),
                new Wait(100000)
        ));
    }
}
