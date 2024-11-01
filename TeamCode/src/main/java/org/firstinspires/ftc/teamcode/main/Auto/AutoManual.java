package org.firstinspires.ftc.teamcode.main.Auto;

import static org.firstinspires.ftc.teamcode.subsystems.Robot.HardwareDevices.clawAngle;

import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
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

@Autonomous
public class AutoManual extends LinearOpMode {

    @Override
    public void runOpMode() {
        Robot robot = new Robot(this);

        Pose2d beginPose = new Pose2d(0, 0, Math.toRadians(270));
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);
        drive.updatePoseEstimate();

        robot.claw.close();
        sleep(500);
        robot.claw.forwards();
        Robot.HardwareDevices.depositSlide.setTargetPositionTolerance(3);
        Robot.HardwareDevices.depositSlide.setTargetPosition(20);
        Robot.HardwareDevices.depositSlide.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        Robot.HardwareDevices.depositSlide.setPower(DepositSlide.DepositSlidePower.move);
        sleep(750);

        waitForStart();

        Actions.runBlocking(new SequentialAction(
                new ParallelAction(
                        drive.actionBuilder(beginPose)
                                .splineToConstantHeading(new Vector2d(0, -18), Math.toRadians(270.00))
                                .splineToSplineHeading(new Pose2d(31, -6, Math.toRadians(45)), Math.toRadians(0))
                                .build(),
                        new DepositActionHigh()
                ),
                new Wait(1000),
                new ClawOpen()
        ));
    }
}