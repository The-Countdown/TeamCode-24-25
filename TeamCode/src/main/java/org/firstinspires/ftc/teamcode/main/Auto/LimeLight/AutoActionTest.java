package org.firstinspires.ftc.teamcode.main.Auto.LimeLight;

import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.main.Auto.RoadRunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.subsystems.actions.intake.IntakeEsc;
import org.firstinspires.ftc.teamcode.subsystems.actions.intake.LimeLightLineup;
import org.firstinspires.ftc.teamcode.subsystems.actions.outtake.OuttakeHighNet;
import org.firstinspires.ftc.teamcode.subsystems.actions.outtake.OuttakeTransferPrep;

@Autonomous(group = "Auto")
public class AutoActionTest extends LinearOpMode {

    @Override
    public void runOpMode() {
        Robot robot = new Robot(this, new Pose2d(0, 57.11, Math.toRadians(0)));
        robot.limeLight.limeLightInit(0, 100);
        Robot.HardwareDevices.flashLight.enableLed(false);

        robot.intake.rest();
        robot.outtake.rest();

        waitForStart();

        Actions.runBlocking(new SequentialAction(
                new InstantAction(() -> MecanumDrive.PARAMS.timeout = 0.5),
                new InstantAction(() -> Robot.rb.intake.restEsc()),
                new SleepAction(2),
                new InstantAction(() -> Robot.rb.intake.elbow.down()),
                new SleepAction(2),
                new LimeLightLineup(robot),
                new SleepAction(0.5),
                new InstantAction(() -> Robot.rb.intake.arm.down()),
                new SleepAction(0.5),
                new InstantAction(() -> Robot.rb.intake.hand.close()),
                new SleepAction(0.5),
                new InstantAction(() -> Robot.rb.intake.arm.up()),
                new SleepAction(0.5),
                Robot.rb.drive.moveAmount(-1,0,90).build(),
                new SleepAction(3),
                new InstantAction(() -> Robot.HardwareDevices.limelight.stop())
        ));
    }
}