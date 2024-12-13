package org.firstinspires.ftc.teamcode.main.Auto;

import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.subsystems.actions.outtake.OuttakeHighNet;
import org.firstinspires.ftc.teamcode.subsystems.actions.outtake.OuttakeTransferPrep;

@Autonomous(group = "Auto")
public class AutoLeft extends LinearOpMode {

    @Override
    public void runOpMode() {
        Robot robot = new Robot(this, new Pose2d(0, 0, Math.toRadians(0))); //TODO Find

        TrajectoryActionBuilder drop1 = robot.roadRunner.actionBuilder(robot.beginPose)
                .stopAndAdd(new InstantAction(() -> robot.intake.restEsc()))
                .stopAndAdd(new OuttakeHighNet())
                .strafeTo(new Vector2d(0, 0)) //TODO Find
                .stopAndAdd(new InstantAction(() -> robot.intake.elbow.down()))
                .stopAndAdd(new InstantAction(() -> robot.intakeSlide.move(0))) //TODO Find
                .waitSeconds(0.5)
                .stopAndAdd(new OuttakeTransferPrep())
                .endTrajectory();

        TrajectoryActionBuilder grab1 = robot.roadRunner.actionBuilder(new Pose2d(0,0,0)) //TODO Find
                .strafeToLinearHeading(new Vector2d(0, 0),0) //TODO Find
                .stopAndAdd(new InstantAction(() -> Robot.rb.intake.arm.down()))
                .waitSeconds(0.25)
                .stopAndAdd(new InstantAction(() -> Robot.rb.intake.hand.close()))
                .waitSeconds(0.15)
                .stopAndAdd(new InstantAction(() -> Robot.rb.intake.arm.up()))
                .waitSeconds(0.1)
                .endTrajectory();

        TrajectoryActionBuilder drop2 = robot.roadRunner.actionBuilder(new Pose2d(0,0,0)) //TODO Find
                .stopAndAdd(new InstantAction(() -> robot.intake.actOne()))
                .strafeToLinearHeading(new Vector2d(0, 0),0) //TODO Find
                .stopAndAdd(new InstantAction(() -> robot.depositSlide.actTwo()))
                .stopAndAdd(new InstantAction(() -> robot.intakeSlide.move(0))) //TODO Find
                .waitSeconds(1) //TODO Find
                .stopAndAdd(new OuttakeTransferPrep())
                .endTrajectory();

        TrajectoryActionBuilder grab2 = robot.roadRunner.actionBuilder(new Pose2d(0,0,0)) //TODO Find
                .strafeToLinearHeading(new Vector2d(0, 0),0) //TODO Find
                .stopAndAdd(new InstantAction(() -> Robot.rb.intake.arm.down()))
                .waitSeconds(0.25)
                .stopAndAdd(new InstantAction(() -> Robot.rb.intake.hand.close()))
                .waitSeconds(0.15)
                .stopAndAdd(new InstantAction(() -> Robot.rb.intake.arm.up()))
                .waitSeconds(0.1)
                .endTrajectory();

        TrajectoryActionBuilder drop3 = robot.roadRunner.actionBuilder(new Pose2d(0,0,0)) //TODO Find
                .stopAndAdd(new InstantAction(() -> robot.intake.actOne()))
                .strafeToLinearHeading(new Vector2d(0, 0),0) //TODO Find
                .stopAndAdd(new InstantAction(() -> robot.depositSlide.actTwo()))
                .waitSeconds(1) //TODO Find
                .stopAndAdd(new InstantAction(() -> robot.outtake.hand.open()))
                .endTrajectory();

        //TODO: What to do after drop3 and before park?

        TrajectoryActionBuilder park = robot.roadRunner.actionBuilder(new Pose2d(0,0,0)) //TODO Find
                .splineToLinearHeading(new Pose2d(0, 0, Math.toRadians(0)), Math.toRadians(0)) //TODO Find
                .splineToLinearHeading(new Pose2d(0, 0, Math.toRadians(0)), Math.toRadians(0)) //TODO Find
                .endTrajectory();


        robot.intake.rest();
        robot.outtake.rest();

        waitForStart();

        Actions.runBlocking(new SequentialAction(
                drop1.build(),
                grab1.build(),
                drop2.build(),
                grab2.build(),
                drop3.build()
//                park.build()
        ));
    }
}