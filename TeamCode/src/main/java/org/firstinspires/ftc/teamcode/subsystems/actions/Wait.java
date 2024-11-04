package org.firstinspires.ftc.teamcode.subsystems.actions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.SleepAction;

public class Wait implements Action {
    private final long duration;

    public Wait(long duration) {
        this.duration = duration;
    }

    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        double durationInSeconds = duration / 1000.0;
        new SleepAction(durationInSeconds);
        return false;
    }
}
