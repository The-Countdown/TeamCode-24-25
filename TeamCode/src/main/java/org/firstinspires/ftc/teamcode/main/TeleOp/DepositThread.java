package org.firstinspires.ftc.teamcode.main.TeleOp;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

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
            try {
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
                    robot.claw.hand.open();
                } else if (gamepad1.cross) {
                    robot.claw.hand.close();
                    Thread.sleep(250);
                    robot.claw.arm.upLift();
                }

                if (gamepad2.dpad_up) {
                    while (!depositMagnet.isPressed()) {
                        robot.claw.arm.down();
                        robot.claw.hand.close();
                        Thread.sleep(750);
                        depositSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        depositSlide.setPower(-0.7);
                    }
                    robot.claw.hand.open();
                    depositSlide.setPower(0);
                    depositSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                }
            } catch (Exception e) {
                telemetry.addData("Error", e.getMessage());
                telemetry.update();
                throw new RuntimeException(e);
            }
        }
    }
}
