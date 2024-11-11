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

            if (gamepad2.dpad_left) {
                robot.intake.wrist.vertical();
            }
            if (gamepad2.dpad_right) {
                robot.intake.wrist.horizontal();
            }

            while (gamepad2.share) {
                if (gamepad2.cross) {
                    robot.intake.elbow.down();
                }
                if (gamepad2.circle) {
                    robot.intake.elbow.up();
                }
                if (gamepad2.square) {
                    robot.intake.elbow.transfer();
                }
                if (gamepad2.triangle) {
                    robot.intake.elbow.rest();
                }
                if (gamepad2.dpad_left) {
                    robot.intake.hand.open();
                }
                if (gamepad2.dpad_right) {
                    robot.intake.hand.close();
                }
            }

            while (gamepad2.right_bumper) {
                if (gamepad2.cross) {
                    robot.outtake.arm.back();
                }
                if (gamepad2.circle) {
                    robot.outtake.arm.upClip();
                }
                if (gamepad2.square) {
                    robot.outtake.arm.transfer();
                }
                if (gamepad2.triangle) {
                    robot.outtake.arm.rest();
                }
                if (gamepad2.dpad_left) {
                    robot.outtake.hand.open();
                }
                if (gamepad2.dpad_right) {
                    robot.outtake.hand.close();
                }
            }
            while (gamepad2.left_bumper) {
                if (gamepad2.cross) {
                    robot.outtake.arm.upLift();
                }
                if (gamepad2.circle) {
                }
                if (gamepad2.square) {
                }
                if (gamepad2.triangle) {
                }
                if (gamepad2.dpad_left) {
                    robot.outtake.wrist.vertical();
                }
                if (gamepad2.dpad_right) {
                    robot.outtake.wrist.horizontal();
                }
            }
        }
    }
}