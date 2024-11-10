package org.firstinspires.ftc.teamcode.subsystems.actions.intake;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

import org.firstinspires.ftc.teamcode.subsystems.Robot;

public class IntakeGround implements Action {
    private final Robot robot;

    public IntakeGround(Robot robot) {
        this.robot = robot;
    }

    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        robot.intakeSlide.ground();
        while (Robot.HardwareDevices.intakeSlideL.isBusy() || Robot.HardwareDevices.intakeSlideR.isBusy()) {
            if (!robot.safeSleep(10)) {
                return true;
            }
        }
        if (!robot.safeSleep(100)) {
            return true;
        }
        return false;
    }
}

