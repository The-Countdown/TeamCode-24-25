package org.firstinspires.ftc.teamcode.main.TeleOp;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

public class LimeLightThread extends Robot.HardwareDevices implements Runnable {
    private final LinearOpMode opMode;
    private final Robot robot;
    private final Gamepad gamepad1;
    private final Gamepad gamepad2;
    public LimeLightThread(LinearOpMode opMode, Robot robot) {
        this.opMode = opMode;
        this.robot = robot;
        this.gamepad1 = opMode.gamepad1;
        this.gamepad2 = opMode.gamepad2;

    }
    @Override
    public void run() {
        while (opMode.opModeIsActive()) {
            if (gamepad2.cross) {
                double orientation = 0;
                orientation = robot.limeLight.getBlockOrientation();

                opMode.telemetry.addData("Orientation", orientation);
                opMode.telemetry.update();
            }
        }
    }
}
