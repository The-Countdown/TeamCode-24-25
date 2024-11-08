package org.firstinspires.ftc.teamcode.main.Auto;

import static org.firstinspires.ftc.teamcode.subsystems.Robot.currentPose;
import static org.firstinspires.ftc.teamcode.subsystems.Robot.rb;

import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystems.actions.ToPose;
import org.firstinspires.ftc.teamcode.subsystems.actions.UpdatePose;

@Autonomous
public class AutoTesting extends LinearOpMode {

    @Override
    public void runOpMode() {
        rb.updatePose();

        TrajectoryActionBuilder toBasket = rb.dreadDrive.actionBuilder(currentPose)
                .splineToLinearHeading(new Pose2d(15, 0, Math.toRadians(-45)), Math.toRadians(-45))
                .splineToLinearHeading(new Pose2d(7, 31, Math.toRadians(-45)), Math.toRadians(-45));

        TrajectoryActionBuilder toFirstSample = rb.dreadDrive.actionBuilder(currentPose)
                .splineToLinearHeading(new Pose2d(15, -10, Math.toRadians(90)), Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(54, -3, Math.toRadians(90)), Math.toRadians(0));
        waitForStart();

        Actions.runBlocking(new ParallelAction(
                new SequentialAction(
                        new ToPose(new Pose2d(15,15,0))
                ),
                new UpdatePose(this)
        ));
    }
}