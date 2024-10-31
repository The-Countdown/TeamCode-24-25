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
import org.firstinspires.ftc.teamcode.subsystems.actions.SpecimenAutoBar;

@Autonomous
public class AutoTest extends LinearOpMode {

    @Override
    public void runOpMode() {
        Robot robot = new Robot(hardwareMap, telemetry, this);

        Pose2d beginPose = new Pose2d(12, 32, Math.toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

//        claw.setPosition(Claw.ClawPosition.closed);
        clawAngle.setPosition(Claw.ClawPosition.horizontal);

        waitForStart();

        Actions.runBlocking(new SequentialAction(
                new ParallelAction(
                        new SequentialAction(
                                drive.actionBuilder(beginPose)
                                        .splineTo(new Vector2d(12.00, 62.00), Math.toRadians(90.00))
                                        .build()
                        ),
                        new SequentialAction(
                                new SpecimenAutoBar()
                        )
                )
        ));
    }
}
