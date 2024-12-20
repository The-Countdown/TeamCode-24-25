package org.firstinspires.ftc.teamcode.main.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.subsystems.DepositSlide;
import org.firstinspires.ftc.teamcode.subsystems.Robot;


public class DepositThread extends Robot.HardwareDevices implements Runnable {
    private final LinearOpMode opMode;
    private final Gamepad gamepad1;
    private final Gamepad gamepad2;
    private final Robot robot;

    public DepositThread(LinearOpMode opMode, Robot robot) {
        this.opMode = opMode;
        this.gamepad1 = opMode.gamepad1;
        this.gamepad2 = opMode.gamepad2;
        this.robot = robot;
    }

    @Override
    public void run() {
        boolean wasRightBumperPressed = false;
        boolean toggleStateRB = false;

        while (opMode.opModeIsActive()) {
            if (gamepad2.circle) {
                robot.depositSlide.condensedMilk();
            }

            if (gamepad2.square) {
                robot.depositSlide.specimenGrab();
            } else if (gamepad2.triangle) {
                robot.depositSlide.specimenHang();
            }

            if (gamepad1.y) {
                robot.outtake.arm.upClip();
                robot.outtake.wrist.horizontal();
                robot.outtake.hand.open();
            }

            boolean isRightBumperPressed = gamepad1.right_bumper;

            if (isRightBumperPressed && !wasRightBumperPressed) {
                toggleStateRB = !toggleStateRB;

                if (toggleStateRB) {
                    robot.outtake.hand.open();
                } else {
                    robot.outtake.hand.close();
                }
            }
            wasRightBumperPressed = isRightBumperPressed;

            if (gamepad2.dpad_down) {
                try {
                    robot.outtake.rest();
                    Thread.sleep(500);
                    Robot.HardwareDevices.depositSlide.setPower(-0.4);
                    Robot.HardwareDevices.depositSlide.setTargetPosition(-5000);
                    double startTime = System.currentTimeMillis();
                    while (!TeleOp.depositMagnetPressed && ((System.currentTimeMillis() - startTime) < 5000)) {
                        Thread.sleep(10);
                    }
                    robot.depositSlide.stop();
                    robot.depositSlide.retract();
                    Robot.HardwareDevices.depositSlide.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
                    Robot.HardwareDevices.depositSlide.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
                    Robot.HardwareDevices.depositSlide.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                    TeleOp.depositMagnetPressed = true;
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }
            }

            int yStickRInt = (int) (gamepad2.right_stick_y * 15);
            if (gamepad2.right_stick_y != 0) {
                int currentPosition = depositSlide.getTargetPosition();

                if (currentPosition >= 5 && gamepad2.right_stick_y > 0) {
                    depositSlide.setTargetPosition(currentPosition - yStickRInt);
                    depositSlide.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                    depositSlide.setPower(DepositSlide.DepositSlidePower.move);
                } else if (currentPosition <= 2550 && gamepad2.right_stick_y < 0) {
                    depositSlide.setTargetPosition(currentPosition - yStickRInt);
                    depositSlide.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                    depositSlide.setPower(DepositSlide.DepositSlidePower.move);
                } else {
                    depositSlide.setTargetPosition(depositSlide.getTargetPosition());
                }
            }
        }
    }
}

