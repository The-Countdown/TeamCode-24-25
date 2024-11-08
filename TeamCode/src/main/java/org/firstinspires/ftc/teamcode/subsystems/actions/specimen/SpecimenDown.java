package org.firstinspires.ftc.teamcode.subsystems.actions.specimen;

import static org.firstinspires.ftc.teamcode.subsystems.Robot.rb;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

public class SpecimenDown implements Action {
    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        rb.depositSlide.specimenWall();
        rb.outtake.hand.open();
        return false;
    }
}
