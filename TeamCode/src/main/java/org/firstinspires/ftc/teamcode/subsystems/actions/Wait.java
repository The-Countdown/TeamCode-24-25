package org.firstinspires.ftc.teamcode.subsystems.actions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

public class Wait implements Action {
    private final long duration;
    private boolean initialized = false;

    public Wait(long duration) {
        this.duration = duration;
    }

    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        if (!initialized) {
            try {
                Thread.sleep(duration);
                initialized = true;
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
                throw new RuntimeException(e);
            }
        }
        return initialized;
    }
}
