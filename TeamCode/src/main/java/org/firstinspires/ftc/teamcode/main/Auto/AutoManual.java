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
public class AutoManual extends LinearOpMode {

    @Override
    public void runOpMode() {
        Robot robot = new Robot(this);

        Pose2d beginPose = new Pose2d(0, 0, Math.toRadians(270));
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);
        drive.updatePoseEstimate();

        waitForStart();

        robot.intake.down();

        Actions.runBlocking(new SequentialAction(
                drive.actionBuilder(beginPose)
                        .splineToConstantHeading(new Vector2d(0, -18), Math.toRadians(270.00))
                        .setReversed(true)
                        .splineToSplineHeading(new Pose2d(31, -6, Math.toRadians(45)), Math.toRadians(225))
                        .build()
        ));
    }
}