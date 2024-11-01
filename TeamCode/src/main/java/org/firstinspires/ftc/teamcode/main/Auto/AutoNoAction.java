package org.firstinspires.ftc.teamcode.main.Auto;

import static org.firstinspires.ftc.teamcode.subsystems.Robot.HardwareDevices.clawAngle;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.main.Auto.RoadRunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

@Autonomous
public class AutoNoAction extends LinearOpMode {

    @Override
    public void runOpMode() {
        Robot robot = new Robot(this);

        Pose2d beginPose = new Pose2d(12, 72, Math.toRadians(270));
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);
        drive.updatePoseEstimate();

        waitForStart();

        Actions.runBlocking(new SequentialAction(
                drive.actionBuilder(beginPose)
                .splineTo(new Vector2d(12.00, 34.00), Math.toRadians(270.00))
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(26.56, 52.41), Math.toRadians(270.00))
                .splineToSplineHeading(new Pose2d(35.85, 26.15, Math.toRadians(0.00)), Math.toRadians(260.00))
                .waitSeconds(0.25)
                .splineToConstantHeading(new Vector2d(40, 26.15), Math.toRadians(0.00))
                .waitSeconds(0.5)
                .strafeToConstantHeading(new Vector2d(40, 35))
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(57.88, 56.86, Math.toRadians(225)), Math.toRadians(45))
                .setReversed(false)
                .splineToLinearHeading(new Pose2d(45, 26.15, Math.toRadians(0.00)), Math.toRadians(0))
                .waitSeconds(0.25)
                .splineToConstantHeading(new Vector2d(50, 26.15), Math.toRadians(0.00))
                .waitSeconds(0.5)
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(57.88, 56.86, Math.toRadians(225)), Math.toRadians(45))
                .setReversed(false)
                .splineToLinearHeading(new Pose2d(55, 26.15, Math.toRadians(0.00)), Math.toRadians(0))
                .waitSeconds(0.25)
                .splineToConstantHeading(new Vector2d(60, 26.15), Math.toRadians(0.00))
                .waitSeconds(0.5)
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(57.88, 56.86, Math.toRadians(225)), Math.toRadians(45))
                .build()
        ));
    }
}