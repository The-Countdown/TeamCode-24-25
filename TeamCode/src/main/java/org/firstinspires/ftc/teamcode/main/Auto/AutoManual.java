package org.firstinspires.ftc.teamcode.main.Auto;

import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.main.Auto.RoadRunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.DepositSlide;
import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.subsystems.actions.Wait;
import org.firstinspires.ftc.teamcode.subsystems.actions.claw.ClawOpen;
import org.firstinspires.ftc.teamcode.subsystems.actions.deposit.DepositActionHigh;
import org.firstinspires.ftc.teamcode.subsystems.actions.deposit.DepositCondense;
import org.firstinspires.ftc.teamcode.subsystems.actions.intake.IntakeCondense;
import org.firstinspires.ftc.teamcode.subsystems.actions.intake.IntakeGround;

@Autonomous
public class AutoManual extends LinearOpMode {

    @Override
    public void runOpMode() {
        Robot robot = new Robot(this);

        Pose2d beginPose = new Pose2d(0, 0, Math.toRadians(0));
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);
        drive.updatePoseEstimate();

        TrajectoryActionBuilder toPlace = drive.actionBuilder(beginPose)
                .splineToLinearHeading(new Pose2d(15, 0, Math.toRadians(-45)), Math.toRadians(-45))
                .splineToLinearHeading(new Pose2d(7, 31, Math.toRadians(-45)), Math.toRadians(-45));

        TrajectoryActionBuilder toFirstSample = toPlace.fresh()
                .splineToLinearHeading(new Pose2d(15, -10, Math.toRadians(90)), Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(54, -3, Math.toRadians(90)), Math.toRadians(0));

        sleep(500);
        Robot.HardwareDevices.depositSlide.setTargetPositionTolerance(3);
        Robot.HardwareDevices.depositSlide.setTargetPosition(20);
        Robot.HardwareDevices.depositSlide.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        Robot.HardwareDevices.depositSlide.setPower(DepositSlide.DepositSlidePower.move);
        sleep(750);

        waitForStart();

        robot.intake.down();
        sleep(100);
        robot.claw.close();
        sleep(100);
        robot.claw.forwards();

        Actions.runBlocking(new SequentialAction(
                toPlace.build(),
                new DepositActionHigh(),
                new ClawOpen(),
                new Wait(200),
                new DepositCondense(),
                toFirstSample.build(),
                new IntakeGround(),
                new Wait(2000),
                new IntakeCondense(),
                toPlace.build(),
                new DepositActionHigh(),
                new ClawOpen(),
                new Wait(200),
                new DepositCondense()
        ));
    }
}