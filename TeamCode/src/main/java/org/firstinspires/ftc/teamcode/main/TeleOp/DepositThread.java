package org.firstinspires.ftc.teamcode.main.TeleOp;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.subsystems.Robot;


public class DepositThread extends Robot.HardwareDevices implements Runnable {
    private final String depSequences;
    private final HardwareMap hardwareMap;
    private final Gamepad gamepad2;
    private final LinearOpMode opMode;
    private final Robot robot;

    public DepositThread(String depSequences, HardwareMap hardwareMap, Gamepad gamepad2, LinearOpMode opMode, Robot robot) {
        this.depSequences = depSequences;
        this.hardwareMap = hardwareMap;
        this.gamepad2 = gamepad2;
        this.opMode = opMode;
        this.robot = robot;
    }

    @Override
    public void run() {
        while (opMode.opModeIsActive()) {
            try {
                if (gamepad2.square) {
                    robot.depositSlide.deposit();
                }
                if (gamepad2.circle) {
                    robot.depositSlide.condense();
                }
            } catch (Exception e) {
                telemetry.addData("Error", e.getMessage());
                telemetry.update();
                throw new RuntimeException(e);
            }
        }
    }
}
