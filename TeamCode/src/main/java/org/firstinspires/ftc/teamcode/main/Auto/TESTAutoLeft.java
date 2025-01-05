package org.firstinspires.ftc.teamcode.main.Auto;

import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.subsystems.actions.Wait;
import org.firstinspires.ftc.teamcode.subsystems.actions.intake.IntakeEsc;
import org.firstinspires.ftc.teamcode.subsystems.actions.intake.IntakePickUp;
import org.firstinspires.ftc.teamcode.subsystems.actions.intake.IntakeWait;
import org.firstinspires.ftc.teamcode.subsystems.actions.outtake.OuttakeActTwo;
import org.firstinspires.ftc.teamcode.subsystems.actions.outtake.OuttakeHighNet;
import org.firstinspires.ftc.teamcode.subsystems.actions.outtake.OuttakeTransferPrep;

public class TESTAutoLeft extends LinearOpMode {

    @Override
    public void runOpMode() {
        Robot robot = new Robot(this, new Pose2d(0, 0, Math.toRadians(0)));
        Pose2d basket = new Pose2d(4, 29, Math.toRadians(315));

        TrajectoryActionBuilder toBasket = robot.roadRunner.actionBuilder(robot.beginPose)
                .splineToLinearHeading(new Pose2d(21, 11.6, Math.toRadians(315)), Math.toRadians(315)) //go forwards angle 45
                .splineToLinearHeading(basket, Math.toRadians(315)); //go to basket

        TrajectoryActionBuilder toFirstSample = robot.roadRunner.actionBuilder(basket)
                .splineToLinearHeading(new Pose2d(28, 17, Math.toRadians(0)), Math.toRadians(0)); //go to first sample

        TrajectoryActionBuilder toBasketTwo = robot.roadRunner.actionBuilder(new Pose2d(28, 17, Math.toRadians(0)))
                .splineToLinearHeading(basket, Math.toRadians(0)); //go to basket

        TrajectoryActionBuilder toSecondSample = robot.roadRunner.actionBuilder(basket)
                .splineToLinearHeading(new Pose2d(28, 17, Math.toRadians(0)), Math.toRadians(0)); //go to second sample

        TrajectoryActionBuilder toBasketThree = robot.roadRunner.actionBuilder(new Pose2d(28, 17, Math.toRadians(0)))
                .splineToLinearHeading(basket, Math.toRadians(0)); //go to basket

        TrajectoryActionBuilder toThirdSample = robot.roadRunner.actionBuilder(basket)
                .splineToLinearHeading(new Pose2d(28, 17, Math.toRadians(0)), Math.toRadians(0)); //go to third sample

        TrajectoryActionBuilder toBasketFour = robot.roadRunner.actionBuilder(new Pose2d(28, 17, Math.toRadians(0)))
                .splineToLinearHeading(basket, Math.toRadians(0)); //go to basket

        TrajectoryActionBuilder toPark = robot.roadRunner.actionBuilder(basket)
                .splineToLinearHeading(new Pose2d(0, 0, Math.toRadians(0)), Math.toRadians(0)) //go near park and spin
                .splineToLinearHeading(new Pose2d(0, 0, Math.toRadians(0)), Math.toRadians(0)); //go to park

        robot.intake.rest();
        robot.outtake.rest();

        waitForStart();

        Actions.runBlocking(new SequentialAction(
                new IntakeEsc(),
                new OuttakeHighNet(),
                toBasket.build(),
                new Wait(1000),
                new InstantAction(() -> robot.outtake.hand.open()),
                new Wait(100),
                new OuttakeTransferPrep(),
                new InstantAction(() -> robot.intake.elbow.down()),
                toFirstSample.build(),
                new IntakePickUp(),
                new InstantAction(() -> robot.intakeSlide.retract()),
                new InstantAction(() -> robot.intake.elbow.up()),
                new InstantAction(() -> robot.intake.arm.transfer()),
                toBasketTwo.build(),
                new Wait(1000),
                new InstantAction(() -> robot.intake.hand.close()),
                new Wait(250),
                new OuttakeActTwo(),
                new Wait(2500),
                new InstantAction(() -> robot.outtake.hand.open()),
                new Wait(100),
                new OuttakeTransferPrep(),
                new InstantAction(() -> robot.intake.arm.up()),
                new Wait(100),
                new InstantAction(() -> robot.intake.elbow.down()),
                new InstantAction(() -> robot.intakeSlide.moveTo(550)),
                new IntakeWait(),
                toSecondSample.build(),
                new IntakePickUp(),
                new InstantAction(() -> robot.intakeSlide.retract()),
                new InstantAction(() -> robot.intake.elbow.up()),
                new InstantAction(() -> robot.intake.arm.transfer()),
                toBasketThree.build(),
                new Wait(1000),
                new InstantAction(() -> robot.intake.hand.close()),
                new Wait(250),
                new OuttakeActTwo(),
                new Wait(2500),
                new InstantAction(() -> robot.outtake.hand.open()),
                new Wait(100),
                new OuttakeTransferPrep(),
                new InstantAction(() -> robot.intake.arm.up()),
                new Wait(100),
                new InstantAction(() -> robot.intake.elbow.down()),
                new InstantAction(() -> robot.intakeSlide.moveTo(550)),
                new IntakeWait(),
                toThirdSample.build(),
                new IntakePickUp(),
                new InstantAction(() -> robot.intakeSlide.retract()),
                new InstantAction(() -> robot.intake.elbow.up()),
                new InstantAction(() -> robot.intake.arm.transfer()),
                toBasketFour.build(),
                new Wait(1000),
                new InstantAction(() -> robot.intake.hand.close()),
                new Wait(250),
                new OuttakeActTwo(),
                new Wait(1750),
                new InstantAction(() -> robot.intake.elbow.up()),
                new Wait(250),
                new InstantAction(() -> robot.intake.rest()),
                new Wait(750),
                new InstantAction(() -> robot.outtake.hand.open()),
                new Wait(100),
                new OuttakeTransferPrep(),
                toPark.build()
        ));
    }
}