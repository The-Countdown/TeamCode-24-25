package org.firstinspires.ftc.teamcode.subsystems.actions.intake;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

import org.firstinspires.ftc.teamcode.subsystems.Robot;

public class IntakePrep implements Action {
    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        Robot.rb.intake.elbow.down();
        Robot.rb.intake.wrist.autoRight();
        return false;
    }
}



