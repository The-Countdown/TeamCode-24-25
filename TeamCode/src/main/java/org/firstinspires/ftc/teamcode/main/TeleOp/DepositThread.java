package org.firstinspires.ftc.teamcode.main.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.subsystems.Robot;


public class DepositThread extends Robot.HardwareDevices implements Runnable {
    private final LinearOpMode opMode;
    private final Gamepad gamepad1;
    private final Gamepad gamepad2;
    private final Robot robot;

    public DepositThread(LinearOpMode opMode) {
        this.opMode = opMode;
        this.gamepad1 = opMode.gamepad1;
        this.gamepad2 = opMode.gamepad2;
        robot = Robot.rb;
    }

    @Override
    public void run() {
        while (opMode.opModeIsActive()) {
            if (gamepad2.circle && (depositSlide.getCurrentPosition() > 75)) {
                robot.depositSlide.condense();
            } else if (gamepad2.circle && (depositSlide.getCurrentPosition() < 75)) {
                robot.depositSlide.depositHigh();
            } else if (gamepad2.share && gamepad2.circle) {
                robot.depositSlide.depositLow();
            }

            if (gamepad2.square) {
                robot.depositSlide.specimenGrab();
            } else if (gamepad2.triangle) {
                robot.depositSlide.specimenHang();
            }

            if (gamepad1.circle) {
                robot.outtake.hand.open();
            } else if (gamepad1.cross) {
                robot.outtake.hand.close();
                if (!robot.safeSleep(250)) {
                    return;
                }
                robot.outtake.arm.upLift();
            }

            if (gamepad2.dpad_up) {
                while (!depositMagnet.isPressed()) {
                    robot.outtake.arm.down();
                    robot.outtake.hand.close();
                    if (!robot.safeSleep(750)) {
                        return;
                    }
                    depositSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    depositSlide.setPower(-0.7);
                }
                robot.outtake.hand.open();
                depositSlide.setPower(0);
                depositSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
        }
    }
}
