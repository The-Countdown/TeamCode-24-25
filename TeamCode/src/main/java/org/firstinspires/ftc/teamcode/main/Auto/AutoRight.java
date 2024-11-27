package org.firstinspires.ftc.teamcode.main.Auto;

import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.main.Auto.RoadRunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.subsystems.actions.Wait;
import org.firstinspires.ftc.teamcode.subsystems.actions.intake.IntakeWait;
import org.firstinspires.ftc.teamcode.subsystems.actions.outtake.OuttakeWait;
import org.firstinspires.ftc.teamcode.subsystems.actions.outtake.OuttakeCondense;
import org.firstinspires.ftc.teamcode.subsystems.actions.outtake.OuttakeCondenseEnd;
import org.firstinspires.ftc.teamcode.subsystems.actions.outtake.OuttakePreloadEsc;
import org.firstinspires.ftc.teamcode.subsystems.actions.outtake.OuttakeSpecimen;

@Autonomous
public class AutoRight extends LinearOpMode {

    @Override
    public void runOpMode() {
        Robot robot = new Robot(this, new Pose2d(0, -14, 0));

        TrajectoryActionBuilder toSubmersibleFromStart = robot.roadRunner.actionBuilder(robot.beginPose)
                .strafeTo(new Vector2d(45, -14));

        TrajectoryActionBuilder toBackItUp = robot.roadRunner.actionBuilder(new Pose2d(45, -14, 0))
                .strafeTo(new Vector2d(29.4, -14));

        TrajectoryActionBuilder toPushSample = robot.roadRunner.actionBuilder(new Pose2d(29.4, -14, 0))
                .splineToConstantHeading(new Vector2d(29.4, -51), 0)
                .splineToConstantHeading(new Vector2d(65, -51), 0)
                .splineToConstantHeading(new Vector2d(65, -70), 0)
                .splineToConstantHeading(new Vector2d(27, -70), 0)
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(27, -70, Math.toRadians(180)), Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(6.5, -70, Math.toRadians(180)), Math.toRadians(180));

        TrajectoryActionBuilder toSubmersibleFromSpecimenFirst = robot.roadRunner.actionBuilder(new Pose2d(6.5, -70, Math.toRadians(180)))
                .setReversed(true)
                .strafeToLinearHeading(new Vector2d(37.5, -6), Math.toRadians(0));

        TrajectoryActionBuilder toIntoSubFirst = robot.roadRunner.actionBuilder(new Pose2d(37.5, -6, Math.toRadians(0)))
                .strafeTo(new Vector2d(58, -6));

        TrajectoryActionBuilder toBackItUpTwo = robot.roadRunner.actionBuilder(new Pose2d(58, -6, 0))
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(30, -6), 0);

        TrajectoryActionBuilder toSecondSpecimen = robot.roadRunner.actionBuilder(new Pose2d(30, -6, 0))
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(21, -60, Math.toRadians(180)), Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(4.3, -60, Math.toRadians(180)), Math.toRadians(180));

        TrajectoryActionBuilder toSubmersibleFromSpecimenSecond = robot.roadRunner.actionBuilder(new Pose2d(4.3, -60, Math.toRadians(180)))
                .setReversed(true)
                .strafeToLinearHeading(new Vector2d(36.5, 0), Math.toRadians(0));

        TrajectoryActionBuilder toIntoSubSecond = robot.roadRunner.actionBuilder(new Pose2d(36.5, 0, Math.toRadians(0)))
                .strafeTo(new Vector2d(47, 0));

        TrajectoryActionBuilder toBackItUpThree = robot.roadRunner.actionBuilder(new Pose2d(47, 0, Math.toRadians(0)))
                .strafeTo(new Vector2d(30, 0));

        robot.intake.rest();
        robot.outtake.rest();

        waitForStart();

        Actions.runBlocking(new SequentialAction(
                new InstantAction(() -> robot.intakeSlide.move(500)),
                new IntakeWait(),
                new OuttakePreloadEsc(),
                new InstantAction(() -> robot.intakeSlide.retract()),
                new InstantAction(() -> robot.outtake.hand.close()),
                toSubmersibleFromStart.build(),
                new InstantAction(() -> robot.depositSlide.specimenBarAltDown()),
                new Wait(100),
                new InstantAction(() -> robot.outtake.hand.halfOpen()),
                new Wait(300),
                new InstantAction(() -> robot.outtake.hand.close()),
                toBackItUp.build(),
                new InstantAction(() -> robot.outtake.hand.open()),
                new Wait(250),
                new OuttakeSpecimen(),
                toPushSample.build(),
                new InstantAction(() -> robot.outtake.hand.close()),
                new Wait(300),
                new InstantAction(() -> robot.outtake.arm.upLift()),
                new Wait(200),
                new InstantAction(() -> robot.depositSlide.move(1110)),
                toSubmersibleFromSpecimenFirst.build(),
                new InstantAction(() -> robot.depositSlide.specimenBarClip()),
                toIntoSubFirst.build(),
                new InstantAction(() -> robot.outtake.hand.open()),
                new Wait(250),
                toBackItUpTwo.build(),
                new OuttakeSpecimen(),
                toSecondSpecimen.build(),
                new InstantAction(() -> robot.outtake.hand.close()),
                new Wait(300),
                new InstantAction(() -> robot.outtake.arm.upLift()),
                new Wait(200),
                new InstantAction(() -> robot.depositSlide.move(1090)),
                toSubmersibleFromSpecimenSecond.build(),
                new InstantAction(() -> robot.depositSlide.specimenBarClip()),
                toIntoSubSecond.build(),
                new InstantAction(() -> robot.outtake.hand.open()),
                toBackItUpThree.build(),
                new OuttakeCondenseEnd(),
                new Wait(5000)
        ));
    }
}
