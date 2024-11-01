package org.firstinspires.ftc.teamcode.subsystems.actions.claw;

import static org.firstinspires.ftc.teamcode.subsystems.Robot.HardwareDevices.claw;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

public class ClawOpen implements Action {
    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        try {
            Robot.rb.claw.open();
            Thread.sleep(500);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        return true;
    }
}

