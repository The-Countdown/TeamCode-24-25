package org.firstinspires.ftc.teamcode.main.Auto.LimeLight;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystems.Robot;

@Autonomous(name = "LimeLightLineup")
public class LimeLightLineup extends LinearOpMode {

    @Override
    public void runOpMode() {
        Robot robot = new Robot(this, new Pose2d(0, 57.11, Math.toRadians(0)));
        robot.intake.arm.up();

        waitForStart();

        while (opModeIsActive()) {
            robot.limeLight.goToLimelightPos(0.1, 0.1, 5);
        }
    }
}
