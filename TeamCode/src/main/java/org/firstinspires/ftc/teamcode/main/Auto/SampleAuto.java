package org.firstinspires.ftc.teamcode.main.Auto;

import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystems.LimeLight;
import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.subsystems.TeleOpPoseUpdater;
import org.firstinspires.ftc.teamcode.subsystems.actions.Wait;
import org.firstinspires.ftc.teamcode.subsystems.actions.intake.IntakeEsc;
import org.firstinspires.ftc.teamcode.subsystems.actions.outtake.OuttakeCondense;
import org.firstinspires.ftc.teamcode.subsystems.actions.outtake.OuttakeHighNet;

@Autonomous(name = "Sample Auto")
public class SampleAuto extends LinearOpMode {
    @Override
    public void runOpMode() {
        Robot robot = new Robot(this, new Pose2d(0, 0, 0));
        robot.intake.rest();
        robot.outtake.rest();
        robot.limeLight.limeLightInit(LimeLight.Pipelines.Yellow, 100);

        TrajectoryActionBuilder toBasket = robot.roadRunner.actionBuilder(robot.beginPose)
                .splineToLinearHeading(new Pose2d(5, 31, Math.toRadians(-55)), 3.14/2);

        TrajectoryActionBuilder toFirstSample = toBasket.fresh()
                .splineToLinearHeading(new Pose2d(16, 26, Math.toRadians(-23)), 0);

        TrajectoryActionBuilder toSecondSample = toBasket.fresh()
                .splineToLinearHeading(new Pose2d(20, 31, Math.toRadians(0)), 0);

        TrajectoryActionBuilder toThirdSample = toBasket.fresh()
                .splineToLinearHeading(new Pose2d(36, 24, Math.toRadians(53)), 0);

        telemetry.addData("Color", Robot.color);
        telemetry.update();

        waitForStart();

        TeleOpPoseUpdater teleOpPoseUpdater = new TeleOpPoseUpdater();
        Thread teleOpPoseUpdaterThread = new Thread(teleOpPoseUpdater);
        teleOpPoseUpdaterThread.start();

        Actions.runBlocking(new SequentialAction(
                new IntakeEsc(),
                new OuttakeHighNet(),
                toBasket.build(),
                new Wait(500),
                new InstantAction(() -> robot.outtake.hand.open()),
                new Wait(300),
                new OuttakeCondense(),
                new InstantAction(() -> robot.intakeSlide.moveTo(1130)),
                new InstantAction(() -> robot.intake.down()),
                new InstantAction(() -> robot.intake.arm.up()),
                toFirstSample.build(),
                new InstantAction(() -> robot.limeLight.pickUp()),
                new InstantAction(() -> robot.intakeSlide.retract()),
                new Wait(500),
                new InstantAction(() -> robot.intake.actOne()),
                toBasket.build(),
                new InstantAction(() -> robot.depositSlide.actTwo()),
                new Wait(600),
                new InstantAction(() -> robot.outtake.hand.open()),
                new Wait(300),
                new OuttakeCondense(),
                new InstantAction(() -> robot.intakeSlide.moveTo(750)),
                new InstantAction(() -> robot.intake.down()),
                new InstantAction(() -> robot.intake.arm.up()),
                toSecondSample.build(),
                new InstantAction(() -> robot.limeLight.pickUp()),
                new InstantAction(() -> robot.intakeSlide.retract()),
                new Wait(500),
                new InstantAction(() -> robot.intake.actOne()),
                toBasket.build(),
                new InstantAction(() -> robot.depositSlide.actTwo()),
                new Wait(600),
                new InstantAction(() -> robot.outtake.hand.open()),
                new Wait(300),
                new OuttakeCondense(),
                new InstantAction(() -> robot.intakeSlide.moveTo(500)),
                new InstantAction(() -> robot.intake.down()),
                new InstantAction(() -> robot.intake.arm.up()),
                toThirdSample.build(),
                new InstantAction(() -> robot.limeLight.pickUp()),
                new InstantAction(() -> robot.intakeSlide.retract()),
                new Wait(500),
                new InstantAction(() -> robot.intake.actOne()),
                toBasket.build(),
                new InstantAction(() -> robot.depositSlide.actTwo()),
                new Wait(600),
                new InstantAction(() -> robot.outtake.hand.open()),
                new InstantAction(() -> robot.intake.rest()),
                new Wait(300),
                new OuttakeCondense(),
                new Wait(1000)
        ));
    }
}
