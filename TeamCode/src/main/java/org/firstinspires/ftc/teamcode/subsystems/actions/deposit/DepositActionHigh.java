package org.firstinspires.ftc.teamcode.subsystems.actions.deposit;

import static org.firstinspires.ftc.teamcode.subsystems.Robot.rb;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.subsystems.actions.Wait;

public class DepositActionHigh implements Action {
    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        rb.claw.hand.close();
        rb.depositSlide.highBasket();
        new Wait(300);
        rb.claw.arm.back();
        rb.claw.elbow.horizontal();
        while (Robot.HardwareDevices.depositSlide.isBusy()) {
            new Wait(10);
        }
        return false;
    }
}

