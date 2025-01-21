package org.firstinspires.ftc.teamcode.main.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
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
        while (opMode.opModeIsActive()) {
            if (!robot.driveAvailable) continue;

            if (gamepad2.circle) {
                robot.depositSlide.condensedMilk();
            }

            if (gamepad2.square) {
                robot.depositSlide.specimenGrab();
            } else if (gamepad2.triangle){
                while (gamepad2.triangle) {
                    robot.depositSlide.specimenHang();
                    robot.outtake.wrist.horizontalFlip();
                }
                robot.outtake.arm.back();
            }

            if (gamepad1.y) {
                robot.outtake.arm.upClip();
                robot.outtake.wrist.horizontal();
                robot.outtake.hand.open();
                robot.depositSlide.retract();
            }

            if (gamepad1.right_bumper) {
                robot.outtake.hand.close();
            }

            if (gamepad1.left_bumper) {
                robot.outtake.hand.open();
            }

            if (gamepad2.dpad_down) {
                robot.depositSlide.magRetract();
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

