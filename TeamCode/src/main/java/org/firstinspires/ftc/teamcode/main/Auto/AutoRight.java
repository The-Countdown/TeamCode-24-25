package org.firstinspires.ftc.teamcode.main.Auto;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.main.Auto.RoadRunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

@Autonomous
public class AutoRight extends LinearOpMode {

    @Override
    public void runOpMode() {
        Robot robot = new Robot(this);
        Pose2d beginPose = new Pose2d(0, 0, -1);
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);
        drive.updatePoseEstimate();

        TrajectoryActionBuilder toSubmersible = drive.actionBuilder(beginPose)
                .strafeTo(new Vector2d(43, 0))

                .strafeTo(new Vector2d(32, 0))

                .strafeTo(new Vector2d(32, -55))
                .strafeTo(new Vector2d(75, -55))
                .strafeTo(new Vector2d(75, -71))
                .strafeTo(new Vector2d(10, -69))
                .strafeTo(new Vector2d(74, -69))
                .strafeTo(new Vector2d(72, -87))
                .strafeTo(new Vector2d(11, -85));

        robot.intake.rest();
        robot.outtake.rest();

        waitForStart();

        Actions.runBlocking(new SequentialAction(
                toSubmersible.build()
//                new IntakeExtend(robot),
//                new IntakeWait(robot),
//                new OuttakePreloadEsc(robot),
//                new IntakeRetract(robot),
//                new OuttakeClawHalfOpen(robot),
//                new Wait(robot,700),
//                new OuttakeClawClose(robot),
//                new Wait(robot, 300000)
        ));
    }
}
