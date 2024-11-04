package org.firstinspires.ftc.teamcode.subsystems.actions.specimen;

import static org.firstinspires.ftc.teamcode.subsystems.Robot.rb;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

public class SpecimenUp implements Action {
    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        rb.depositSlide.specimenHang();
        return false;
    }
}
