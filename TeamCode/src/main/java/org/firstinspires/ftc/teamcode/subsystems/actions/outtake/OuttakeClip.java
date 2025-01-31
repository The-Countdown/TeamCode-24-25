package org.firstinspires.ftc.teamcode.subsystems.actions.outtake;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

import org.firstinspires.ftc.teamcode.subsystems.Robot;

public class OuttakeClip implements Action {
    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        try {
            Robot.rb.outtake.arm.back();
            Thread.sleep(300);
            Robot.rb.outtake.hand.open();
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        return false;
    }
}



