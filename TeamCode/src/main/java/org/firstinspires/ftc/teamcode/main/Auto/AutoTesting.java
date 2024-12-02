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
                .waitSeconds(0.75)
                .strafeTo(new Vector2d(45, -14))
                .stopAndAdd(new OuttakeClip())
                .stopAndAdd(new InstantAction(() -> Robot.rb.intake.elbow.down()))
                .stopAndAdd(new InstantAction(() -> Robot.rb.intake.wrist.autoRight()))
                .endTrajectory();

        TrajectoryActionBuilder grab1 = robot.roadRunner.actionBuilder(new Pose2d(45, -14, Math.toRadians(180)))
                .afterTime(0.5, new InstantAction(() -> robot.intakeSlide.move(1400)))
                .afterDisp(15, new OuttakeSpecimen())
                .splineToLinearHeading(new Pose2d(21.1, -44.2, Math.toRadians(320)), Math.toRadians(320))
                .stopAndAdd(new InstantAction(() -> Robot.rb.intake.arm.down()))
                .waitSeconds(0.15)
                .stopAndAdd(new InstantAction(() -> Robot.rb.intake.hand.close()))
                .waitSeconds(0.15)
                .stopAndAdd(new InstantAction(() -> Robot.rb.intake.arm.up()))
                .waitSeconds(0.1)
                .endTrajectory();

        TrajectoryActionBuilder drop1 = robot.roadRunner.actionBuilder(new Pose2d(21.1, -44.2, Math.toRadians(320)))
                .stopAndAdd(new InstantAction(() -> Robot.rb.outtake.arm.upLift()))
                .splineToLinearHeading(new Pose2d(21.2, -44.2, Math.toRadians(240)), Math.toRadians(240))
                .endTrajectory();

        TrajectoryActionBuilder grab2 = robot.roadRunner.actionBuilder(new Pose2d(21.2, -44.2, Math.toRadians(240)))
                .splineToLinearHeading(new Pose2d(19.5, -58.5, Math.toRadians(320)), Math.toRadians(320))
                .stopAndAdd(new InstantAction(() -> Robot.rb.intake.arm.down()))
                .waitSeconds(0.15)
                .stopAndAdd(new InstantAction(() -> Robot.rb.intake.hand.close()))
                .waitSeconds(0.15)
                .stopAndAdd(new InstantAction(() -> Robot.rb.intake.arm.up()))
                .waitSeconds(0.1)
                .endTrajectory();

        TrajectoryActionBuilder drop2 = robot.roadRunner.actionBuilder(new Pose2d(19.5, -58.5, Math.toRadians(320)))
                .splineToLinearHeading(new Pose2d(19, -59, Math.toRadians(240)), Math.toRadians(240))
                .endTrajectory();

        TrajectoryActionBuilder grab3 = robot.roadRunner.actionBuilder(new Pose2d(19, -59, Math.toRadians(240)))
                .splineToLinearHeading(new Pose2d(18, -59, Math.toRadians(320)), Math.toRadians(320))
                .splineToLinearHeading(new Pose2d(18.2, -70.6, Math.toRadians(320)), Math.toRadians(320))
                .stopAndAdd(new InstantAction(() -> Robot.rb.intake.arm.down()))
                .waitSeconds(0.15)
                .stopAndAdd(new InstantAction(() -> Robot.rb.intake.hand.close()))
                .waitSeconds(0.15)
                .stopAndAdd(new InstantAction(() -> Robot.rb.intake.arm.up()))
                .waitSeconds(0.1)
                .endTrajectory();

        TrajectoryActionBuilder drop3 = robot.roadRunner.actionBuilder(new Pose2d(18.2, -70.6, Math.toRadians(320)))
                .stopAndAdd(new InstantAction(() -> robot.intakeSlide.move(300)))
                .waitSeconds(0.15)
                .splineToLinearHeading(new Pose2d(10, -65, Math.toRadians(225)), Math.toRadians(225))
                .endTrajectory();

        TrajectoryActionBuilder specimen1 = robot.roadRunner.actionBuilder(new Pose2d(10, -65, Math.toRadians(225)))
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(15, -60, Math.toRadians(180)), Math.toRadians(180))
                .stopAndAdd(new InstantAction(() -> Robot.rb.intake.rest()))
                .stopAndAdd(new InstantAction(() -> Robot.rb.outtake.arm.upClip()))
                .waitSeconds(1)
                .splineToConstantHeading(new Vector2d(2.5, -60), Math.toRadians(180))
                .endTrajectory();

        TrajectoryActionBuilder clip2 = robot.roadRunner.actionBuilder(new Pose2d(2.5, -60, Math.toRadians(180)))
                .setReversed(true)
                .stopAndAdd(new InstantAction(() -> Robot.rb.outtake.arm.back()))
                .strafeToConstantHeading(new Vector2d(35, -6))
                .strafeToConstantHeading(new Vector2d(45, -6))
                .stopAndAdd(new OuttakePreloadEsc())
//                .waitSeconds(0.5)
                .stopAndAdd(new OuttakeClip())
                .waitSeconds(0.15)
                .endTrajectory();

        TrajectoryActionBuilder specimen2 = robot.roadRunner.actionBuilder(new Pose2d(45, -6, Math.toRadians(180)))
                .stopAndAdd(new OuttakeSpecimen())
                .splineToConstantHeading(new Vector2d(15, -60), Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(2.5, -60), Math.toRadians(180))
                .endTrajectory();

        TrajectoryActionBuilder clip3 = robot.roadRunner.actionBuilder(new Pose2d(2.5, -60, Math.toRadians(180)))
                .setReversed(true)
                .stopAndAdd(new InstantAction(() -> Robot.rb.outtake.arm.back()))
                .strafeToConstantHeading(new Vector2d(35, -2))
                .strafeToConstantHeading(new Vector2d(45, -2))
                .stopAndAdd(new OuttakePreloadEsc())
//                .waitSeconds(0.5)
                .stopAndAdd(new OuttakeClip())
                .waitSeconds(0.15)
                .endTrajectory();

        TrajectoryActionBuilder specimen3 = robot.roadRunner.actionBuilder(new Pose2d(45, -2, Math.toRadians(180)))
                .stopAndAdd(new OuttakeSpecimen())
                .splineToConstantHeading(new Vector2d(15, -60), Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(2.5, -60), Math.toRadians(180))
                .endTrajectory();

//        TrajectoryActionBuilder clip4 = robot.roadRunner.actionBuilder(new Pose2d(2.5, -60, Math.toRadians(180)))
//                .setReversed(true)
//                .stopAndAdd(new InstantAction(() -> Robot.rb.outtake.arm.back()))
//                .strafeToConstantHeading(new Vector2d(35, 2))
//                .strafeToConstantHeading(new Vector2d(45, 2))
//                .stopAndAdd(new OuttakePreloadEsc())
////                .waitSeconds(0.5)
//                .stopAndAdd(new OuttakeClip())
//                .waitSeconds(0.15)
//                .endTrajectory();

        robot.intake.rest();
        robot.outtake.rest();

        waitForStart();

        Actions.runBlocking(new SequentialAction(
                new InstantAction(() -> MecanumDrive.PARAMS.timeout = 0.1),
                clip1.build(),
                new SleepAction(0.15),
                new InstantAction(() -> MecanumDrive.PARAMS.timeout = 0.4),
                grab1.build(),
                new InstantAction(() -> MecanumDrive.PARAMS.timeout = 0),
                drop1.build(),
                new InstantAction(() -> Robot.rb.intake.hand.open()),
                new SleepAction(0.15),
                new InstantAction(() -> MecanumDrive.PARAMS.timeout = 0.4),
                grab2.build(),
                new InstantAction(() -> MecanumDrive.PARAMS.timeout = 0),
                drop2.build(),
                new InstantAction(() -> Robot.rb.intake.hand.open()),
                new SleepAction(0.15),
                new InstantAction(() -> MecanumDrive.PARAMS.timeout = 0.4),
                grab3.build(),
                new InstantAction(() -> MecanumDrive.PARAMS.timeout = 0),
                drop3.build(),
                new InstantAction(() -> Robot.rb.intake.hand.open()),
                new SleepAction(0.15),
                new InstantAction(() -> Robot.rb.intake.elbow.up()),
                new InstantAction(() -> Robot.rb.intakeSlide.retract()),
                new InstantAction(() -> MecanumDrive.PARAMS.timeout = 0.2),
                specimen1.build(),
                new InstantAction(() -> Robot.rb.outtake.hand.close()),
                new SleepAction(0.3),
                new InstantAction(() -> Robot.rb.outtake.arm.back()),
                new InstantAction(() -> robot.depositSlide.specimenBar()),
                new SleepAction(0.15),
                new InstantAction(() -> MecanumDrive.PARAMS.timeout = 0),
                clip2.build(),
                new InstantAction(() -> MecanumDrive.PARAMS.timeout = 0.2),
                specimen2.build(),
                new InstantAction(() -> Robot.rb.outtake.hand.close()),
                new SleepAction(0.3),
                new InstantAction(() -> Robot.rb.outtake.arm.back()),
                new InstantAction(() -> robot.depositSlide.specimenBar()),
                new SleepAction(0.15),
                new InstantAction(() -> MecanumDrive.PARAMS.timeout = 0),
                clip3.build(),
                new InstantAction(() -> MecanumDrive.PARAMS.timeout = 0.2),
                specimen3.build(),
                new InstantAction(() -> Robot.rb.outtake.hand.close()),
                new SleepAction(0.3),
                new InstantAction(() -> Robot.rb.outtake.arm.back()),
                new InstantAction(() -> robot.depositSlide.specimenBar()),
                new SleepAction(0.15),
                new InstantAction(() -> MecanumDrive.PARAMS.timeout = 0),
//                clip4.build(),
                new SleepAction(999999999)
        ));
    }
}
