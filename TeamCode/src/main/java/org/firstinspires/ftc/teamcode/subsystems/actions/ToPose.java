package org.firstinspires.ftc.teamcode.subsystems.actions;

import static org.firstinspires.ftc.teamcode.subsystems.Robot.rb;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.SleepAction;

public class ToPose implements Action {
    private final double x;
    private final double y;
    private final double angle;

    public ToPose(double x, double y, double angle) {
        this.x = x;
        this.y = y;
        this.angle = angle;
    }

    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {

        return false;
    }
}
