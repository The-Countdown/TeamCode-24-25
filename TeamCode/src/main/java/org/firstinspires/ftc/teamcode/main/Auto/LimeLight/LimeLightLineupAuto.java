package org.firstinspires.ftc.teamcode.main.Auto.LimeLight;

import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystems.Robot;

@Autonomous(name = "LimeLightLineup")
public class LimeLightLineupAuto extends LinearOpMode {

    @Override
    public void runOpMode() {
        Robot robot = new Robot(this, new Pose2d(0, 57.11, Math.toRadians(0)));
        robot.limeLight.limeLightInit(0,30);
        robot.intake.arm.up();

        waitForStart();

        Actions.runBlocking(new SequentialAction(
                robot.limeLight.goToLimelightPos(0.1, 0.1, 2.5).build(),
                new InstantAction(() -> robot.intakeSlide.moveTo(Robot.rb.intakeSlide.avg() + 100))
        ));
        Robot.HardwareDevices.limelight.stop();
    }
}
