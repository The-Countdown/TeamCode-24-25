package org.firstinspires.ftc.teamcode.main.Auto;

import com.acmerobotics.roadrunner.InstantAction;
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
import org.firstinspires.ftc.teamcode.subsystems.actions.intake.IntakeEsc;
import org.firstinspires.ftc.teamcode.subsystems.actions.outtake.OuttakeClip;
import org.firstinspires.ftc.teamcode.subsystems.actions.outtake.OuttakePreloadEsc;
import org.firstinspires.ftc.teamcode.subsystems.actions.outtake.OuttakeSpecimen;

@Autonomous
public class AutoTesting extends LinearOpMode {

    @Override
    public void runOpMode() {
        Robot robot = new Robot(this, new Pose2d(0, -14, Math.toRadians(180)));

        TrajectoryActionBuilder clip1 = robot.roadRunner.actionBuilder(robot.beginPose)
                .afterTime(0, new OuttakePreloadEsc())
                .afterDisp(40, new IntakeEsc())
                .strafeTo(new Vector2d(32, -14))
                .waitSeconds(1)
                .strafeTo(new Vector2d(45, -14))
                .stopAndAdd(new OuttakeClip())
                .stopAndAdd(new InstantAction(() -> Robot.rb.intake.elbow.down()))
                .stopAndAdd(new InstantAction(() -> Robot.rb.intake.wrist.autoRight()))
                .endTrajectory();

        TrajectoryActionBuilder grab1 = robot.roadRunner.actionBuilder(new Pose2d(45, -14, Math.toRadians(180)))
                .afterTime(0.5, new InstantAction(() -> robot.intakeSlide.move(1400)))
                .afterDisp(15, new OuttakeSpecimen())
                .splineToLinearHeading(new Pose2d(21.1, -44.4, Math.toRadians(320)), Math.toRadians(320))
                .stopAndAdd(new InstantAction(() -> Robot.rb.intake.arm.down()))
                .waitSeconds(0.15)
                .stopAndAdd(new InstantAction(() -> Robot.rb.intake.hand.close()))
                .waitSeconds(0.15)
                .stopAndAdd(new InstantAction(() -> Robot.rb.intake.arm.up()))
                .waitSeconds(0.1)
                .endTrajectory();

        TrajectoryActionBuilder drop1 = robot.roadRunner.actionBuilder(new Pose2d(21.1, -44.4, Math.toRadians(320)))
                .stopAndAdd(new InstantAction(() -> Robot.rb.outtake.arm.upLift()))
                .splineToLinearHeading(new Pose2d(21.2, -44.4, Math.toRadians(240)), Math.toRadians(240))
                .endTrajectory();

        TrajectoryActionBuilder grab2 = robot.roadRunner.actionBuilder(new Pose2d(21.2, -44.4, Math.toRadians(240)))
                .splineToLinearHeading(new Pose2d(19.5, -59.1, Math.toRadians(320)), Math.toRadians(320))
                .waitSeconds(0.5)
                .stopAndAdd(new InstantAction(() -> Robot.rb.intake.arm.down()))
                .waitSeconds(0.15)
                .stopAndAdd(new InstantAction(() -> Robot.rb.intake.hand.close()))
                .waitSeconds(0.15)
                .stopAndAdd(new InstantAction(() -> Robot.rb.intake.arm.up()))
                .waitSeconds(0.1)
                .endTrajectory();

        TrajectoryActionBuilder drop2 = robot.roadRunner.actionBuilder(new Pose2d(19.3, -59.1, Math.toRadians(320)))
                .splineToLinearHeading(new Pose2d(19, -59.5, Math.toRadians(240)), Math.toRadians(240))
                .endTrajectory();

        TrajectoryActionBuilder grab3 = robot.roadRunner.actionBuilder(new Pose2d(19, -59.5, Math.toRadians(240)))
                .splineToLinearHeading(new Pose2d(18, -58.6, Math.toRadians(320)), Math.toRadians(320))
                .splineToLinearHeading(new Pose2d(18.4, -71.2, Math.toRadians(320)), Math.toRadians(320))
                .stopAndAdd(new InstantAction(() -> Robot.rb.intake.arm.down()))
                .waitSeconds(0.15)
                .stopAndAdd(new InstantAction(() -> Robot.rb.intake.hand.close()))
                .waitSeconds(0.15)
                .stopAndAdd(new InstantAction(() -> Robot.rb.intake.arm.up()))
                .waitSeconds(0.1)
                .endTrajectory();

        TrajectoryActionBuilder drop3 = robot.roadRunner.actionBuilder(new Pose2d(18.4, -71.2, Math.toRadians(320)))
                .stopAndAdd(new InstantAction(() -> robot.intakeSlide.move(300)))
                .waitSeconds(0.15)
                .splineToLinearHeading(new Pose2d(10, -65, Math.toRadians(225)), Math.toRadians(225))
                .endTrajectory();

        TrajectoryActionBuilder specimen1 = robot.roadRunner.actionBuilder(new Pose2d(18, -65, Math.toRadians(225)))
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(18, -60, Math.toRadians(180)), Math.toRadians(0))
                .stopAndAdd(new InstantAction(() -> Robot.rb.intake.rest()))
                .waitSeconds(1.5)
                .stopAndAdd(new InstantAction(() -> Robot.rb.outtake.arm.upClip()))
                .splineToLinearHeading(new Pose2d(6, -60, Math.toRadians(180)), Math.toRadians(0))
                .endTrajectory();

        TrajectoryActionBuilder toSubmersibleFromSpecimenFirst = robot.roadRunner.actionBuilder(new Pose2d(6, -65, Math.toRadians(180)))
                .setReversed(true)
                .stopAndAdd(new InstantAction(() -> Robot.rb.outtake.arm.back()))
                .strafeToLinearHeading(new Vector2d(37.5, -2), Math.toRadians(180));

        TrajectoryActionBuilder clip2 = robot.roadRunner.actionBuilder(new Pose2d(37.5, -2, Math.toRadians(180)))
                .afterTime(0, new OuttakePreloadEsc())
                .waitSeconds(1)
                .lineToX(45)
                .stopAndAdd(new OuttakeClip())
                .stopAndAdd(new InstantAction(() -> Robot.rb.outtake.hand.close()))
                .setReversed(true)
                .lineToX(35)
                .endTrajectory();

//        TrajectoryActionBuilder toIntoSubFirst = robot.roadRunner.actionBuilder(new Pose2d(37.5, -6, Math.toRadians(180)))
//                .strafeTo(new Vector2d(58, -6));
//
//        TrajectoryActionBuilder toBackItUpTwo = robot.roadRunner.actionBuilder(new Pose2d(58, -6, Math.toRadians(180)))
//                .setReversed(true)
//                .splineToConstantHeading(new Vector2d(30, -6), 0);

        TrajectoryActionBuilder toSecondSpecimen = robot.roadRunner.actionBuilder(new Pose2d(30, -6, Math.toRadians(180)))
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(21, -60, Math.toRadians(180)), Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(4.3, -60, Math.toRadians(180)), Math.toRadians(0));

        TrajectoryActionBuilder toSubmersibleFromSpecimenSecond = robot.roadRunner.actionBuilder(new Pose2d(4.3, -60, Math.toRadians(0)))
                .setReversed(true)
                .strafeToLinearHeading(new Vector2d(36.5, 0), Math.toRadians(180));

        TrajectoryActionBuilder toIntoSubSecond = robot.roadRunner.actionBuilder(new Pose2d(36.5, 0, Math.toRadians(180)))
                .strafeTo(new Vector2d(47, 0));

        TrajectoryActionBuilder toBackItUpThree = robot.roadRunner.actionBuilder(new Pose2d(47, 0, Math.toRadians(180)))
                .strafeTo(new Vector2d(30, 0));

        robot.intake.rest();
        robot.outtake.rest();

        waitForStart();

        Actions.runBlocking(new SequentialAction(
                new InstantAction(() -> MecanumDrive.PARAMS.timeout = 0.3),
                clip1.build(),
                new SleepAction(0.15),
                new InstantAction(() -> MecanumDrive.PARAMS.timeout = 2),
                grab1.build(),
                drop1.build(),
                new InstantAction(() -> Robot.rb.intake.hand.open()),
                new SleepAction(0.15),
                grab2.build(),
                drop2.build(),
                new InstantAction(() -> Robot.rb.intake.hand.open()),
                new SleepAction(0.15),
                grab3.build(),
                drop3.build(),
                new InstantAction(() -> Robot.rb.intake.hand.open()),
                new SleepAction(0.15),
                new InstantAction(() -> Robot.rb.intake.elbow.up()),
                new InstantAction(() -> Robot.rb.intakeSlide.retract()),
                specimen1.build(),
                new InstantAction(() -> MecanumDrive.PARAMS.timeout = 0.2),
                new InstantAction(() -> Robot.rb.outtake.hand.close()),
                new SleepAction(0.15),
                new InstantAction(() -> Robot.rb.outtake.arm.upClip()),
                toSubmersibleFromSpecimenFirst.build(),
                new SleepAction(0.15),
                clip2.build(),
                new SleepAction(2),
                toSecondSpecimen.build(),
                new InstantAction(() -> Robot.rb.outtake.hand.close()),
                new SleepAction(0.15)
        ));
    }
}
