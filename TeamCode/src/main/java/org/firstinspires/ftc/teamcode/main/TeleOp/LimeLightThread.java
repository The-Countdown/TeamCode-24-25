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
            double orientation = 0;
            orientation = Math.abs(robot.limeLight.getBlockOrientation());

            if (orientation != 0) {
                opMode.telemetry.addData("orientation prescale", orientation);
                orientation /= 1800;
                opMode.telemetry.addData("orientation", orientation);
                opMode.telemetry.update();
                Robot.HardwareDevices.intakeClawAngle.setPosition(Intake.IntakePosition.wristHorizontal + orientation);
            }
        }
    }
}
