package org.firstinspires.ftc.teamcode.subsystems.actions.intake;

import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

import org.firstinspires.ftc.teamcode.subsystems.Robot;

public class IntakeEsc implements Action {
    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        Robot.rb.intakeSlide.move(500);
        Robot.rb.intake.restEsc();
        return false;
    }
}
