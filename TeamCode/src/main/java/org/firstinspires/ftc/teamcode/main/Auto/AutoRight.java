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
import org.firstinspires.ftc.teamcode.subsystems.actions.intake.LimeLightLineup;
import org.firstinspires.ftc.teamcode.subsystems.actions.outtake.OuttakeClip;
import org.firstinspires.ftc.teamcode.subsystems.actions.outtake.OuttakePreloadEsc;
import org.firstinspires.ftc.teamcode.subsystems.actions.outtake.OuttakeSpecimen;
import org.firstinspires.ftc.teamcode.subsystems.actions.outtake.OuttakeSpecimenAlt;

@Autonomous(group = "Auto")
public class AutoRight extends LinearOpMode {

    @Override
    public void runOpMode() {
        Robot robot = new Robot(this, new Pose2d(0, -14, Math.toRadians(180)));

        TrajectoryActionBuilder clip1 = robot.roadRunner.actionBuilder(robot.beginPose)
                .afterTime(0, new OuttakePreloadEsc())
                .afterTime(0.75, new InstantAction(() -> Robot.rb.depositSlide.move(190)))
                .afterDisp(40, new IntakeEsc())
                .strafeTo(new Vector2d(56, -2))
                .stopAndAdd(new OuttakeClip())
                .stopAndAdd(new InstantAction(() -> Robot.rb.intake.elbow.down()))
                .stopAndAdd(new InstantAction(() -> Robot.rb.intake.wrist.autoRight()))
                .endTrajectory();

        TrajectoryActionBuilder grab1 = robot.roadRunner.actionBuilder(new Pose2d(49, -2, Math.toRadians(180)))
                .afterTime(0.75, new InstantAction(() -> robot.intakeSlide.moveTo(1400)))
                .afterDisp(15, new OuttakeSpecimenAlt())
                .splineToLinearHeading(new Pose2d(16.1, -47, Math.toRadians(320)), Math.toRadians(320))
                .stopAndAdd(new LimeLightLineup(robot))
                .stopAndAdd(new InstantAction(() -> Robot.rb.intake.arm.down()))
                .waitSeconds(0.25)
                .stopAndAdd(new InstantAction(() -> Robot.rb.intake.hand.close()))
                .waitSeconds(0.15)
                .stopAndAdd(new InstantAction(() -> Robot.rb.intake.arm.up()))
                .waitSeconds(0.1)
                .endTrajectory();

        TrajectoryActionBuilder drop1 = robot.roadRunner.actionBuilder(new Pose2d(18.1, -47, Math.toRadians(320)))
                .splineToLinearHeading(new Pose2d(18, -47, Math.toRadians(225)), Math.toRadians(225))
                .endTrajectory();

        TrajectoryActionBuilder grab2 = robot.roadRunner.actionBuilder(new Pose2d(18, -47, Math.toRadians(225)))
                .splineToLinearHeading(new Pose2d(16, -63, Math.toRadians(320)), Math.toRadians(320))
                .stopAndAdd(new LimeLightLineup(robot))
                .stopAndAdd(new InstantAction(() -> Robot.rb.intake.arm.down()))
                .waitSeconds(0.25)
                .stopAndAdd(new InstantAction(() -> Robot.rb.intake.hand.close()))
                .waitSeconds(0.15)
                .stopAndAdd(new InstantAction(() -> Robot.rb.intake.arm.up()))
                .waitSeconds(0.1)
                .endTrajectory();

        TrajectoryActionBuilder drop2 = robot.roadRunner.actionBuilder(new Pose2d(18.1, -63, Math.toRadians(320)))
                .splineToLinearHeading(new Pose2d(18, -63, Math.toRadians(230)), Math.toRadians(230))
                .endTrajectory();

        TrajectoryActionBuilder grab3 = robot.roadRunner.actionBuilder(new Pose2d(18, -63, Math.toRadians(230)))
                .splineToLinearHeading(new Pose2d(16.5, -58, Math.toRadians(320)), Math.toRadians(320))
                .splineToLinearHeading(new Pose2d(16.5, -69.8, Math.toRadians(320)), Math.toRadians(320))
                .stopAndAdd(new LimeLightLineup(robot))
                .stopAndAdd(new InstantAction(() -> Robot.rb.intake.arm.down()))
                .waitSeconds(0.25)
                .stopAndAdd(new InstantAction(() -> Robot.rb.intake.hand.close()))
                .waitSeconds(0.15)
                .stopAndAdd(new InstantAction(() -> Robot.rb.intake.arm.up()))
                .waitSeconds(0.1)
                .endTrajectory();

        TrajectoryActionBuilder drop3 = robot.roadRunner.actionBuilder(new Pose2d(18.5, -69.8, Math.toRadians(320)))
                .stopAndAdd(new InstantAction(() -> robot.depositSlide.move(400)))
                .waitSeconds(0.4)
                .stopAndAdd(new InstantAction(() -> robot.intakeSlide.moveTo(300)))
                .waitSeconds(0.15)
                .splineToLinearHeading(new Pose2d(10, -65, Math.toRadians(225)), Math.toRadians(225))
                .endTrajectory();

        TrajectoryActionBuilder specimen1 = robot.roadRunner.actionBuilder(new Pose2d(10, -65, Math.toRadians(230)))
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(18, -60, Math.toRadians(180)), Math.toRadians(180))
                .stopAndAdd(new InstantAction(() -> Robot.rb.intake.rest()))
                .stopAndAdd(new InstantAction(() -> Robot.rb.outtake.arm.upClip()))
                .stopAndAdd(new InstantAction(() -> Robot.rb.depositSlide.magRetract()))
                .waitSeconds(0.25)
                .splineToConstantHeading(new Vector2d(0, -60), Math.toRadians(180))
                .endTrajectory();

        TrajectoryActionBuilder clip2 = robot.roadRunner.actionBuilder(new Pose2d(0, -60, Math.toRadians(180)))
                .setReversed(true)
                .strafeToConstantHeading(new Vector2d(35, 0))
                .stopAndAdd(new InstantAction(() -> Robot.rb.outtake.wrist.horizontalFlip()))
                .strafeToConstantHeading(new Vector2d(49, 0))
                .stopAndAdd(new OuttakeClip())
                .waitSeconds(0.15)
                .endTrajectory();

        TrajectoryActionBuilder specimen2 = robot.roadRunner.actionBuilder(new Pose2d(49, 0, Math.toRadians(180)))
                .stopAndAdd(new OuttakeSpecimen())
                .splineToConstantHeading(new Vector2d(15, -60), Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(0, -60), Math.toRadians(180))
                .endTrajectory();

        TrajectoryActionBuilder clip3 = robot.roadRunner.actionBuilder(new Pose2d(0, -60, Math.toRadians(180)))
                .setReversed(true)
                .strafeToConstantHeading(new Vector2d(35, 8))
                .stopAndAdd(new InstantAction(() -> Robot.rb.outtake.wrist.horizontalFlip()))
                .strafeToConstantHeading(new Vector2d(49, 8))
                .stopAndAdd(new OuttakeClip())
                .waitSeconds(0.15)
                .endTrajectory();

        TrajectoryActionBuilder after = robot.roadRunner.actionBuilder(new Pose2d(49, 8, Math.toRadians(180)))
                .strafeToConstantHeading(new Vector2d(35, 8))
                .waitSeconds(0.15)
                .stopAndAdd(new InstantAction(() -> Robot.rb.outtake.arm.upClip()))
                .stopAndAdd(new InstantAction(() -> Robot.rb.depositSlide.magRetract()))
                .endTrajectory();

        TrajectoryActionBuilder specimen3 = robot.roadRunner.actionBuilder(new Pose2d(35, 8, Math.toRadians(180)))
                .stopAndAdd(new OuttakeSpecimen())
                .splineToConstantHeading(new Vector2d(15, -60), Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(0, -60), Math.toRadians(180))
                .endTrajectory();

        TrajectoryActionBuilder clip4 = robot.roadRunner.actionBuilder(new Pose2d(0, -60, Math.toRadians(180)))
                .setReversed(true)
                .strafeToConstantHeading(new Vector2d(35, 2))
                .stopAndAdd(new InstantAction(() -> Robot.rb.outtake.wrist.horizontalFlip()))
                .strafeToConstantHeading(new Vector2d(49, 2))
                .stopAndAdd(new OuttakeClip())
                .waitSeconds(0.15)
                .endTrajectory();

        robot.intake.rest();
        robot.outtake.rest();
        robot.limeLight.limeLightInit(2,100);

        waitForStart();

        Actions.runBlocking(new SequentialAction(
                new InstantAction(() -> MecanumDrive.PARAMS.timeout = 0.3),
                clip1.build(),
                new SleepAction(0.15),
                new InstantAction(() -> MecanumDrive.PARAMS.timeout = -0.2),
                grab1.build(),
                new InstantAction(() -> MecanumDrive.PARAMS.timeout = 0.1),
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
                new InstantAction(() -> Robot.rb.outtake.hand.close()),
                new SleepAction(0.3),
                new InstantAction(() -> Robot.rb.depositSlide.move(190)),
                new InstantAction(() -> Robot.rb.outtake.arm.upLift()),
                new SleepAction(0.15),
                clip2.build(),
                specimen2.build(),
                new InstantAction(() -> Robot.rb.outtake.hand.close()),
                new SleepAction(0.3),
                new InstantAction(() -> Robot.rb.depositSlide.move(190)),
                new InstantAction(() -> Robot.rb.outtake.arm.upLift()),
                new SleepAction(0.15),
                clip3.build(),
                after.build(),
                specimen3.build(),
                new InstantAction(() -> Robot.rb.outtake.hand.close()),
                new SleepAction(0.3),
                new InstantAction(() -> Robot.rb.depositSlide.move(190)),
                new InstantAction(() -> Robot.rb.outtake.arm.upLift()),
                new SleepAction(0.15),
                clip4.build(),
                new SleepAction(999999999)
        ));
    }
}
