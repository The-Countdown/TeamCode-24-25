package org.firstinspires.ftc.teamcode.subsystems.actions;

import static org.firstinspires.ftc.teamcode.subsystems.Robot.rb;

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
        return !rb.safeSleep(duration);
    }
}
