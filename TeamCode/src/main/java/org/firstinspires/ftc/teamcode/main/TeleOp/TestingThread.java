package org.firstinspires.ftc.teamcode.main.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.subsystems.Robot;


public class TestingThread extends Robot.HardwareDevices implements Runnable {
    private final LinearOpMode opMode;
    private final Gamepad gamepad2;
    private final Robot robot;

    public TestingThread(LinearOpMode opMode, Robot robot) {
        this.opMode = opMode;
        this.gamepad2 = opMode.gamepad2;
        this.robot = robot;
    }

    @Override
    public void run() {
        while (opMode.opModeIsActive()) {
            if (gamepad2.cross) {
                robot.intake.arm.down();
            }
            if (gamepad2.circle) {
                robot.intake.arm.up();
            }
            if (gamepad2.square) {
                robot.intake.arm.transfer();
            }
            if (gamepad2.triangle) {
                robot.intake.arm.rest();
            }
            while (gamepad2.share) {

            }
            while (gamepad2.right_bumper) {

            }
        }
    }
}