package org.firstinspires.ftc.teamcode.main.TeleOp;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.DepositSlide;
import org.firstinspires.ftc.teamcode.subsystems.Robot;


public class DepositThread extends Robot.HardwareDevices implements Runnable {
    private final String depSequences;
    private final HardwareMap hardwareMap;
    private final Gamepad gamepad1;
    private final Gamepad gamepad2;
    private final LinearOpMode opMode;
    private final Robot robot;


    public DepositThread(String depSequences, HardwareMap hardwareMap, Gamepad gamepad2, Gamepad gamepad1, LinearOpMode opMode, Robot robot) {
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
                if (gamepad2.dpad_left) {
                    robot.depositSlide.deposit();
                }
                if (gamepad1.circle && (clawArm.getPosition() == Claw.ClawPosition.back)) {
                    claw.setPosition(Claw.ClawPosition.open);
                }

                if (gamepad2.dpad_right) {
                    robot.depositSlide.condense();
                }

                if ((!depositSlide.isBusy()) && (depositSlide.getTargetPosition() < DepositSlide.DepositSlidePosition.stopTolerance)) {
                    depositSlide.setPower(DepositSlide.DepositSlidePower.stop);
                }

            } catch (Exception e) {
                telemetry.addData("Error", e.getMessage());
                telemetry.update();
                throw new RuntimeException(e);
            }
        }
    }
}
