package org.firstinspires.ftc.teamcode.subsystems.actions.intake;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

import org.firstinspires.ftc.teamcode.subsystems.Robot;

public class IntakeCondense implements Action {
    private final Robot robot;

    public IntakeCondense(Robot robot) {
        this.robot = robot;
    }

    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        robot.outtake.arm.down();
        robot.outtake.hand.close();
        robot.intake.up();
        if (!robot.safeSleep(750)) {
            return true;
        }
        robot.intakeSlide.retract();
        while (Robot.HardwareDevices.intakeSlideL.isBusy() || Robot.HardwareDevices.intakeSlideR.isBusy()) {
            if (!robot.safeSleep(10)) {
                return true;
            }
        }
        robot.intake.down();
        if (!robot.safeSleep(500)) {
            return true;
        }
        robot.outtake.hand.close();
        if (!robot.safeSleep(100)) {
            return true;
        }
        return false;
    }
}

