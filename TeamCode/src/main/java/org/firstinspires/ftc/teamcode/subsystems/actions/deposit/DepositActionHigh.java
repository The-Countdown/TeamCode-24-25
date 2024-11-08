package org.firstinspires.ftc.teamcode.subsystems.actions.deposit;

import static org.firstinspires.ftc.teamcode.subsystems.Robot.rb;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

import org.firstinspires.ftc.teamcode.subsystems.Robot;

public class DepositActionHigh implements Action {
    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        rb.outtake.hand.close();
        rb.depositSlide.highBasket();
        if (!rb.safeSleep(300)) {
            return true;
        }
        rb.outtake.arm.back();
        rb.outtake.wrist.horizontal();
        while (Robot.HardwareDevices.depositSlide.isBusy()) {
            if (!rb.safeSleep(10)) {
                return true;
            }
        }
        return false;
    }
}

