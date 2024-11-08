package org.firstinspires.ftc.teamcode.main.TeleOp;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.subsystems.Robot;


public class DriveThread extends Robot.HardwareDevices implements Runnable {
    private final LinearOpMode opMode;
    private final Gamepad gamepad1;
    private final Robot robot;

    public DriveThread(LinearOpMode opMode) {
        this.opMode = opMode;
        this.gamepad1 = opMode.gamepad1;
        this.robot = Robot.rb;
    }

    @Override
    public void run() {
        while (opMode.opModeIsActive()) {
            if (gamepad1.square) {
                robot.drive.toPose(new Pose2d(15,0,0));
            }
            if (gamepad1.triangle) {
                robot.drive.toPose(new Pose2d(15,15,0));
            }
        }
    }
}
