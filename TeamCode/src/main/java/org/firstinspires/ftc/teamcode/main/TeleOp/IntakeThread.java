package org.firstinspires.ftc.teamcode.main.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSlide;
import org.firstinspires.ftc.teamcode.subsystems.Robot;


public class IntakeThread extends Robot.HardwareDevices implements Runnable {
    private final LinearOpMode opMode;
    private final Gamepad gamepad1;
    private final Gamepad gamepad2;
    private final Robot robot;
    public static boolean snapshot = true;

    public IntakeThread(LinearOpMode opMode, Robot robot) {
        this.opMode = opMode;
        this.gamepad1 = opMode.gamepad1;
        this.gamepad2 = opMode.gamepad2;
        this.robot = robot;
    }

    @Override
    public void run() {
        boolean wasRightBumperPressed = false;
        boolean toggleStateRB = false;
        boolean wasRightTriggerPressed = false;
        boolean wasLeftTriggerPressed = false;
        boolean toggleStateLT = false;

        while (opMode.opModeIsActive()) {
            if (gamepad2.left_bumper) {
                robot.intake.restEsc();
            }

//            if (gamepad2.dpad_right) {
//                robot.intakeSlide.magRetract();
//            }

            if (gamepad2.dpad_up) {
                robot.intake.elbow.up();
            }

            if (gamepad2.share) {
                robot.intake.actOne();
                while (!gamepad2.share) {
                    if (gamepad2.right_bumper || gamepad2.right_trigger > 0.1) {
                        robot.intake.hand.open();
                        robot.intake.down();
                        robot.depositSlide.condensedMilk();
                        break;
                    }
                }
                if (snapshot) {Robot.HardwareDevices.limelight.captureSnapshot(TeleOp.currentDateTime);}
                robot.depositSlide.actTwo();
            }

//            boolean isRightBumperPressed = gamepad2.right_bumper;
//
//            if (isRightBumperPressed && !wasRightBumperPressed) {
//                toggleStateRB = !toggleStateRB;
//
//                if (toggleStateRB) {
//                    robot.intake.hand.close();
//                } else {
//                    robot.intake.hand.open();
//                }
//            }
//            wasRightBumperPressed = isRightBumperPressed;

            boolean isRightTriggerPressed = gamepad2.right_trigger > 0.1;

            if (isRightTriggerPressed && !wasRightTriggerPressed) {
                robot.intake.down();
            } else if (!isRightTriggerPressed && wasRightTriggerPressed) {
                robot.intake.arm.up();
            }

            wasRightTriggerPressed = isRightTriggerPressed;

            boolean isLeftTriggerPressed = gamepad2.left_trigger > 0.1;

            if (isLeftTriggerPressed && !wasLeftTriggerPressed) {
                toggleStateLT = !toggleStateLT;

                if (toggleStateLT) {
                    robot.intake.wrist.vertical();
                } else {
                    robot.intake.wrist.horizontal();
                }
            }

            wasLeftTriggerPressed = isLeftTriggerPressed;

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
                } else {
                    intakeSlideL.setTargetPosition(intakeSlideL.getTargetPosition());
                    intakeSlideR.setTargetPosition(intakeSlideR.getTargetPosition());
                }
            }
        }
    }
}