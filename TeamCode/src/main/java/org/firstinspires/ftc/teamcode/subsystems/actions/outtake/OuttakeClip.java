package org.firstinspires.ftc.teamcode.subsystems.actions.outtake;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

import org.firstinspires.ftc.teamcode.subsystems.Robot;

public class OuttakeClip implements Action {
    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        Robot.rb.depositSlide.specimenBarClip();
        while (!(Robot.HardwareDevices.depositSlide.getCurrentPosition() > 1150)) {
            telemetryPacket.addLine("Waiting...");
        }
        telemetryPacket.clearLines();
        Robot.rb.outtake.hand.open();
        return false;
    }
}



