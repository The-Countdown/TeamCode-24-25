package org.firstinspires.ftc.teamcode.subsystems.actions.deposit;

import static org.firstinspires.ftc.teamcode.subsystems.Robot.HardwareDevices.depositSlide;
import static org.firstinspires.ftc.teamcode.subsystems.Robot.rb;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

public class DepositCondense implements Action {
    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        rb.depositClaw.arm.down();
        rb.depositClaw.hand.close();
        rb.depositClaw.elbow.vertical();
        rb.depositSlide.retract();
        while (depositSlide.isBusy()) {
            if (!rb.safeSleep(10)) {
                return true;
            }
        }
        return false;
    }
}

