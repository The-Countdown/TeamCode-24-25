package org.firstinspires.ftc.teamcode.main.Auto;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystems.Robot;

@Autonomous
public class AutoTest extends LinearOpMode {

    @Override
    public void runOpMode() {
        Robot robot = new Robot(hardwareMap, telemetry, this);

        waitForStart();

        robot.drive.red.basket();
        robot.depositSlide.deposit();

        while (opModeIsActive()) {
            Pose2d currentPos = robot.drive.getRobotPos();
            if (currentPos.position.y > 55 && currentPos.position.y < 57 && currentPos.position.x > 54 && currentPos.position.x < 56) {
                break;
            }
        }

        robot.claw.open();
    }
}
