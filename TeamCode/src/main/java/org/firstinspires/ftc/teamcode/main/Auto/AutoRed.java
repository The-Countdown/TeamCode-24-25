package org.firstinspires.ftc.teamcode.main.Auto;

import static org.firstinspires.ftc.teamcode.subsystems.Robot.HardwareDevices.clawAngle;

import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.main.Auto.RoadRunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.subsystems.actions.Wait;
import org.firstinspires.ftc.teamcode.subsystems.actions.claw.ClawOpen;
import org.firstinspires.ftc.teamcode.subsystems.actions.deposit.DepositActionHigh;
import org.firstinspires.ftc.teamcode.subsystems.actions.deposit.DepositCondense;
import org.firstinspires.ftc.teamcode.subsystems.actions.intake.IntakeCondense;
import org.firstinspires.ftc.teamcode.subsystems.actions.intake.IntakeGround;
import org.firstinspires.ftc.teamcode.subsystems.actions.intake.IntakeSpit;
import org.firstinspires.ftc.teamcode.subsystems.actions.specimen.SpecimenAutoBar;
import org.firstinspires.ftc.teamcode.subsystems.actions.specimen.SpecimenDown;

@Autonomous
public class AutoRed extends LinearOpMode {

    @Override
    public void runOpMode() {
        Robot robot = new Robot(this);

        Pose2d beginPose = new Pose2d(12, 62, Math.toRadians(270));
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);
        drive.updatePoseEstimate();

        clawAngle.setPosition(Claw.ClawPosition.horizontal);

        waitForStart();

        Actions.runBlocking(new SequentialAction(
                new ParallelAction(
                        new SequentialAction(
                                // Drive to high chamber and get ready to place
                                drive.actionBuilder(beginPose)
                                        .splineTo(new Vector2d(-12.00, -34.00), Math.toRadians(90.00))
                                        .build()
                        ),
                        new SequentialAction(
                                new SpecimenAutoBar()
                        )
                ),
                new SequentialAction(
                        new SpecimenDown()
                ),
                new ParallelAction(
                        new SequentialAction(
                                // Back up and spin into neutral samples to get ready to pick up
                                drive.actionBuilder(drive.pose)
                                        .setReversed(true)
                                        .splineToConstantHeading(new Vector2d(-26.56, -52.41), Math.toRadians(90.00))
                                        .splineToSplineHeading(new Pose2d(-35.85, -26.15, Math.toRadians(180.00)), Math.toRadians(100.00))
                                        .waitSeconds(0.25)
                                        .build()
                        ),
                        new SequentialAction(
                                new Wait(500),
                                new IntakeGround()
                        )
                ),
                new SequentialAction(
                        // Goes forwards slightly to intake samples
                        drive.actionBuilder(drive.pose)
                                .splineToConstantHeading(new Vector2d(-40, -26.15), Math.toRadians(180.00))
                                .waitSeconds(0.5)
                                .build()
                ),
                new ParallelAction(
                        new SequentialAction(
                                // Goes to basket to deposit sample
                                drive.actionBuilder(drive.pose)
                                        .strafeToConstantHeading(new Vector2d (-40, -35))
                                        .setReversed(true)
                                        .splineToLinearHeading(new Pose2d(-57.88, -56.86, Math.toRadians(45)), Math.toRadians(225))
                                        .build()
                        ),
                        new SequentialAction(
                                new IntakeCondense(),
                                new IntakeSpit(),
                                new DepositActionHigh()
                        )
                ),
                new SequentialAction(
                        new ClawOpen()
                ),
                new ParallelAction(
                        new SequentialAction(
                                new DepositCondense()
                        ),
                        new SequentialAction(
                                // Goes to second sample
                                drive.actionBuilder(drive.pose)
                                        .setReversed(false)
                                        .splineToLinearHeading(new Pose2d(-45, -26.15, Math.toRadians(180.00)), Math.toRadians(180.00))
                                        .waitSeconds(0.25)
                                        .build()
                        ),
                        new SequentialAction(
                                new Wait(500),
                                new IntakeGround()
                        )
                ),
                new SequentialAction(
                        // Drives into second sample to pick up
                        drive.actionBuilder(drive.pose)
                                .splineToConstantHeading(new Vector2d(-50, -26.15), Math.toRadians(180.00))
                                .waitSeconds(0.5)
                                .build()
                ),
                new ParallelAction(
                        new SequentialAction(
                                // Goes to basket to deposit sample
                                drive.actionBuilder(drive.pose)
                                        .setReversed(true)
                                        .splineToLinearHeading(new Pose2d(-57.88, -56.86, Math.toRadians(45)), Math.toRadians(225))
                                        .build()
                        ),
                        new SequentialAction(
                                new IntakeCondense(),
                                new IntakeSpit(),
                                new DepositActionHigh()
                        )
                ),
                new SequentialAction(
                        new ClawOpen()
                ),
                new ParallelAction(
                        new SequentialAction(
                                new DepositCondense()
                        ),
                        new SequentialAction(
                                // Goes to third sample
                                drive.actionBuilder(drive.pose)
                                        .setReversed(false)
                                        .splineToLinearHeading(new Pose2d(-55, -26.15, Math.toRadians(180.00)), Math.toRadians(180.00))
                                        .waitSeconds(0.25)
                                        .build()
                        ),
                        new SequentialAction(
                                new Wait(500),
                                new IntakeGround()
                        )
                ),
                new SequentialAction(
                        // Drives into third sample to pick up
                        drive.actionBuilder(drive.pose)
                                .splineToConstantHeading(new Vector2d(-60, -26.15), Math.toRadians(180.00))
                                .waitSeconds(0.5)
                                .build()
                ),
                new ParallelAction(
                        new SequentialAction(
                                // Goes to basket to deposit sample
                                drive.actionBuilder(drive.pose)
                                        .setReversed(true)
                                        .splineToLinearHeading(new Pose2d(-57.88, -56.86, Math.toRadians(45)), Math.toRadians(225))
                                        .build()
                        ),
                        new SequentialAction(
                                new IntakeCondense(),
                                new IntakeSpit(),
                                new DepositActionHigh()
                        )
                )
        ));

    }
}
