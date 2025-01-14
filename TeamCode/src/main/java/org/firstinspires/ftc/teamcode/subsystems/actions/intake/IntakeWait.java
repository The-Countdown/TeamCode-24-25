package org.firstinspires.ftc.teamcode.subsystems.actions.intake;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

import org.firstinspires.ftc.teamcode.subsystems.Robot;

public class IntakeWait implements Action {
    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        while (Robot.HardwareDevices.intakeSlideL.isBusy() || Robot.HardwareDevices.intakeSlideR.isBusy()) {
            if (!Robot.rb.safeSleep(10)) {
                return true;
            }
        }
        return false;
    }
}



