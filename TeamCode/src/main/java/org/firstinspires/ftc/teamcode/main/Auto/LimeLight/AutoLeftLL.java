package org.firstinspires.ftc.teamcode.main.Auto.LimeLight;

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
import org.firstinspires.ftc.teamcode.subsystems.actions.intake.LimeLightLineup;
import org.firstinspires.ftc.teamcode.subsystems.actions.outtake.OuttakeHighNet;
import org.firstinspires.ftc.teamcode.subsystems.actions.outtake.OuttakeTransferPrep;

@Autonomous(group = "Auto")
public class AutoLeftLL extends LinearOpMode {

    @Override
    public void runOpMode() {
        Robot robot = new Robot(this, new Pose2d(0, 57.11, Math.toRadians(0)));

        TrajectoryActionBuilder drop1 = robot.roadRunner.actionBuilder(robot.beginPose)
                .stopAndAdd(new InstantAction(() -> robot.intake.restEsc()))
                .stopAndAdd(new OuttakeHighNet())
                .splineToLinearHeading(new Pose2d(10.9, 72.7, Math.toRadians(-45)), Math.toRadians(-45))
                .splineToLinearHeading(new Pose2d(5.1, 88.6, Math.toRadians(-45)), Math.toRadians(-45))
                .stopAndAdd(new InstantAction(() -> robot.intake.elbow.down()))
                .stopAndAdd(new InstantAction(() -> robot.intakeSlide.moveTo(1400)))
                .waitSeconds(0.5)
                .stopAndAdd(new OuttakeTransferPrep())
                .endTrajectory();

        TrajectoryActionBuilder grab1 = robot.roadRunner.actionBuilder(new Pose2d(5.1,88.6, Math.toRadians(-45)))
                .strafeToLinearHeading(new Vector2d(10.7, 72.5),0)
                .stopAndAdd(new LimeLightLineup(robot))
                .stopAndAdd(new InstantAction(() -> Robot.rb.intake.arm.down()))
                .waitSeconds(0.25)
                .stopAndAdd(new InstantAction(() -> Robot.rb.intake.hand.close()))
                .waitSeconds(0.15)
                .stopAndAdd(new InstantAction(() -> Robot.rb.intake.arm.up()))
                .waitSeconds(0.1)
                .endTrajectory();

        TrajectoryActionBuilder drop2 = robot.roadRunner.actionBuilder(new Pose2d(10.7,72.5,0))
                .stopAndAdd(new InstantAction(() -> robot.intake.actOne()))
                .stopAndAdd(new InstantAction(() -> robot.outtake.arm.transfer()))
                .strafeToLinearHeading(new Vector2d(6.1, 87.6), Math.toRadians(-45))
                .stopAndAdd(new InstantAction(() -> robot.depositSlide.actTwo()))
                .stopAndAdd(new InstantAction(() -> robot.intakeSlide.moveTo(1400)))
                .waitSeconds(1) //TODO Find
                .stopAndAdd(new OuttakeTransferPrep())
                .endTrajectory();

        TrajectoryActionBuilder grab2 = robot.roadRunner.actionBuilder(new Pose2d(6.1,87.6,Math.toRadians(-45)))
                .strafeToLinearHeading(new Vector2d(10.4, 86.4),0)
                .stopAndAdd(new LimeLightLineup(robot))
                .stopAndAdd(new InstantAction(() -> Robot.rb.intake.arm.down()))
                .waitSeconds(0.25)
                .stopAndAdd(new InstantAction(() -> Robot.rb.intake.hand.close()))
                .waitSeconds(0.15)
                .stopAndAdd(new InstantAction(() -> Robot.rb.intake.arm.up()))
                .waitSeconds(0.1)
                .endTrajectory();

        TrajectoryActionBuilder drop3 = robot.roadRunner.actionBuilder(new Pose2d(10.4,86.4,0))
                .stopAndAdd(new InstantAction(() -> robot.intake.actOne()))
                .stopAndAdd(new InstantAction(() -> robot.outtake.arm.transfer()))
                .strafeToLinearHeading(new Vector2d(6.1, 87.6), Math.toRadians(-45))
                .stopAndAdd(new InstantAction(() -> robot.depositSlide.actTwo()))
                .waitSeconds(1) //TODO Find
                .stopAndAdd(new InstantAction(() -> robot.outtake.hand.open()))
                .endTrajectory();

        //TODO: What to do after drop3 and before park?

        TrajectoryActionBuilder park = robot.roadRunner.actionBuilder(new Pose2d(6.1,87.6,Math.toRadians(-45)))
                .splineToLinearHeading(new Pose2d(0, 0, Math.toRadians(0)), Math.toRadians(0)) //TODO Find
                .splineToLinearHeading(new Pose2d(0, 0, Math.toRadians(0)), Math.toRadians(0)) //TODO Find
                .endTrajectory();


        robot.intake.rest();
        robot.outtake.rest();

        waitForStart();

        Actions.runBlocking(new SequentialAction(
                new InstantAction(() -> MecanumDrive.PARAMS.timeout = 0.5),
                drop1.build(),
                grab1.build(),
                drop2.build(),
                grab2.build(),
                drop3.build(),
//                park.build()
                new SleepAction(10000000)
        ));
    }
}