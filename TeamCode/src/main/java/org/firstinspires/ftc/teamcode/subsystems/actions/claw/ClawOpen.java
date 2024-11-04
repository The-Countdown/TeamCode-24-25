package org.firstinspires.ftc.teamcode.subsystems.actions.claw;

import static org.firstinspires.ftc.teamcode.subsystems.Robot.rb;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

import org.firstinspires.ftc.teamcode.subsystems.Robot;

public class ClawOpen implements Action {
    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        rb.claw.hand.open();
        return false;
    }
}

