package org.firstinspires.ftc.teamcode.main.TeleOp;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Robot;


public class DepositThread extends Robot.HardwareDevices implements Runnable {
    private final String depSequences;
    private final HardwareMap hardwareMap;
    private final Gamepad gamepad1;
    private final Gamepad gamepad2;
    private final LinearOpMode opMode;
    private final Robot robot;

    public DepositThread(String depSequences, HardwareMap hardwareMap, Gamepad gamepad1, Gamepad gamepad2, LinearOpMode opMode, Robot robot) {
        this.depSequences = depSequences;
        this.hardwareMap = hardwareMap;
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
        this.opMode = opMode;
        this.robot = robot;
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

                if (gamepad1.circle && clawArm.getPosition() == Claw.ClawPosition.upLift) {
                    claw.setPosition(Claw.ClawPosition.open);
                } else if (gamepad1.cross && clawArm.getPosition() == Claw.ClawPosition.upClip) {
                    claw.setPosition(Claw.ClawPosition.closed);
                    Thread.sleep(250);
                    clawArm.setPosition(Claw.ClawPosition.upLift);
                } else if (gamepad1.cross) {
                    claw.setPosition(Claw.ClawPosition.closed);
                } else if (gamepad1.circle) {
                claw.setPosition(Claw.ClawPosition.open);
            }
            } catch (Exception e) {
                telemetry.addData("Error", e.getMessage());
                telemetry.update();
                throw new RuntimeException(e);
            }
        }
    }
}
