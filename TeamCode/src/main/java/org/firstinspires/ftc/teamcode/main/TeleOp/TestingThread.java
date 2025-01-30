package org.firstinspires.ftc.teamcode.main.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.subsystems.DepositSlide;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSlide;
import org.firstinspires.ftc.teamcode.subsystems.Robot;


public class TestingThread extends Robot.HardwareDevices implements Runnable {
    private final LinearOpMode opMode;
    private final Gamepad gamepad2;
    private final Robot robot;
    public boolean isCurrent;

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
                robot.intake.wrist.horizontalFlip();
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
                    robot.intake.rest();
                }
                if (gamepad2.circle) {
                    robot.depositSlide.specimenGrab();
                }
                if (gamepad2.square) {
                    robot.outtake.wrist.horizontalFlip();
                }
                if (gamepad2.triangle) {
                    robot.intake.restEsc();
                }
                if (gamepad2.dpad_left) {
                    robot.outtake.wrist.horizontal();
                }
                if (gamepad2.dpad_right) {
                    robot.outtake.wrist.vertical();
                }
            }

            int yStickRInt = (int) (gamepad2.right_stick_y * 15);
            if (gamepad2.right_stick_y != 0) {
                int currentPosition = depositSlide.getTargetPosition();

                if (currentPosition >= 5 && gamepad2.right_stick_y > 0) {
                    depositSlide.setTargetPosition(currentPosition - yStickRInt);
                    depositSlide.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                    depositSlide.setPower(DepositSlide.DepositSlidePower.move);
                } else if (currentPosition <= DepositSlide.DepositSlidePosition.highBasket && gamepad2.right_stick_y < 0) {
                    depositSlide.setTargetPosition(currentPosition - yStickRInt);
                    depositSlide.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                    depositSlide.setPower(DepositSlide.DepositSlidePower.move);
                }
            } else {
                depositSlide.setTargetPosition(depositSlide.getTargetPosition());
            }

//            int maxPosition = 2550;
//            int minPosition = 0;
//            if (gamepad2.right_stick_y != 0) {
//                double joystickInput = -gamepad2.right_stick_y;
//
//                int targetPosition;
//                if (joystickInput > 0) {
//                    targetPosition = maxPosition;
//                } else {
//                    targetPosition = minPosition;
//                }
//
//                depositSlide.setTargetPosition(targetPosition);
//                depositSlide.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
//                depositSlide.setPower(Math.abs(joystickInput));
//                isCurrent = false;
//            } else if (gamepad2.right_stick_y == 0) {
//                if (!isCurrent) {
//                    isCurrent = true;
//                    depositSlide.setTargetPosition(depositSlide.getCurrentPosition());
//                    depositSlide.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
//                    depositSlide.setPower(DepositSlide.DepositSlidePower.move);
//                }
//                depositSlide.setTargetPosition(depositSlide.getTargetPosition());
//                depositSlide.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
//                depositSlide.setPower(DepositSlide.DepositSlidePower.move);
//            }

            int yStickLInt  = (int) (gamepad2.left_stick_y * 30);
            if (gamepad2.left_stick_y != 0) {
                int averagePosition = (intakeSlideL.getTargetPosition() + intakeSlideR.getTargetPosition()) / 2;

                if (averagePosition >= 5 && gamepad2.left_stick_y > 0) {
                    intakeSlideL.setTargetPosition(intakeSlideL.getTargetPosition() - yStickLInt);
                    intakeSlideR.setTargetPosition(intakeSlideR.getTargetPosition() - yStickLInt);
                    intakeSlideL.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                    intakeSlideR.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                    intakeSlideL.setPower(IntakeSlide.IntakeSlidePower.move);
                    intakeSlideR.setPower(IntakeSlide.IntakeSlidePower.move);
                } else if (averagePosition <= 1500 && gamepad2.left_stick_y < 0) {
                    intakeSlideL.setTargetPosition(intakeSlideL.getTargetPosition() - yStickLInt);
                    intakeSlideR.setTargetPosition(intakeSlideR.getTargetPosition() - yStickLInt);
                    intakeSlideL.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                    intakeSlideR.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                    intakeSlideL.setPower(IntakeSlide.IntakeSlidePower.move);
                    intakeSlideR.setPower(IntakeSlide.IntakeSlidePower.move);
                }
            } else {
                intakeSlideL.setTargetPosition(intakeSlideL.getTargetPosition());
                intakeSlideR.setTargetPosition(intakeSlideR.getTargetPosition());
            }
        }
    }
}