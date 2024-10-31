package org.firstinspires.ftc.teamcode.subsystems.actions.claw;

import static org.firstinspires.ftc.teamcode.subsystems.Robot.HardwareDevices.claw;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

import org.firstinspires.ftc.teamcode.subsystems.Claw;

public class ClawOpen implements Action {
    private boolean initialized = false;

    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        if (!initialized) {
            try {
                claw.setPosition(Claw.ClawPosition.open);
                Thread.sleep(500);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
        }
        initialized = true;
        return initialized;
    }
}

