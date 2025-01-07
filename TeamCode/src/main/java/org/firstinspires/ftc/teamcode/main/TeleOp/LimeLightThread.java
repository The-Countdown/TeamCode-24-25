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
        boolean protection = false;
        robot.intake.arm.up();
        while (opMode.opModeIsActive()) {
            if (!gamepad1.a) {
                protection = false;
                continue;
            }
            if (protection) {
                continue;
            }
            protection = true;

            double orientation = 0;
            orientation = Math.abs(robot.limeLight.getBlockOrientation()) - 90;

            if (orientation != -90) {
                opMode.telemetry.addData("orientation preoffset", orientation);
                double currentServoAngle = (Robot.HardwareDevices.intakeClawAngle.getPosition() - Intake.IntakePosition.wristHorizontal) * 355;
                opMode.telemetry.addData("currentServoAngle", currentServoAngle);
                orientation += currentServoAngle;
                opMode.telemetry.addData("orientation prescale", orientation);
                orientation /= 355;
                opMode.telemetry.addData("orientation", orientation);

                opMode.telemetry.addData("target", Intake.IntakePosition.wristHorizontal - orientation);

                opMode.telemetry.update();
                Robot.HardwareDevices.intakeClawAngle.setPosition(Intake.IntakePosition.wristHorizontal - orientation);
            }
        }
    }
}
