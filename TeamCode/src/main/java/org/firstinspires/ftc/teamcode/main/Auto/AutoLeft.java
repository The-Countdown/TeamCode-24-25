package org.firstinspires.ftc.teamcode.main.Auto;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.main.Auto.RoadRunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

@Autonomous
public class AutoLeft extends LinearOpMode {

    @Override
    public void runOpMode() {
        Robot robot = new Robot(this, new Pose2d(0, 0, 0));

        TrajectoryActionBuilder toBasket = robot.roadRunner.actionBuilder(robot.beginPose)
                .splineToLinearHeading(new Pose2d(15, 0, Math.toRadians(-45)), Math.toRadians(-45))
                .splineToLinearHeading(new Pose2d(7, 31, Math.toRadians(-45)), Math.toRadians(-45));

        TrajectoryActionBuilder toFirstSample = toBasket.fresh()
                .splineToLinearHeading(new Pose2d(15, -10, Math.toRadians(90)), Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(54, -3, Math.toRadians(90)), Math.toRadians(0));

        robot.intake.rest();

        waitForStart();

        Actions.runBlocking(new SequentialAction(

        ));
    }
}