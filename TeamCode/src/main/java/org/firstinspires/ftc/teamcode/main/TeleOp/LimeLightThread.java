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
                lineUp();
            }
        }
    }

    private void lineUp() {
        LLResult result;
        result = limelight.getLatestResult();

        if (result == null) {
            opMode.telemetry.addData("Limelight", "No Targets");
            opMode.telemetry.update();
            return;
        }

        double tx = result.getTx();

        opMode.telemetry.addData("tx", tx);

        Robot.HardwareDevices.intakeClawAngle.setPosition(Intake.IntakePosition.wristHorizontal - (tx / Robot.servoToDegrees));
        opMode.telemetry.addData("angle", Intake.IntakePosition.wristHorizontal - (tx / Robot.servoToDegrees));

        opMode.telemetry.update();
    }
}
