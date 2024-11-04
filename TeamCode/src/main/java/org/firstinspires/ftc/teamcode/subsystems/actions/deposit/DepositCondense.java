package org.firstinspires.ftc.teamcode.subsystems.actions.deposit;

import static org.firstinspires.ftc.teamcode.subsystems.Robot.HardwareDevices.depositSlide;
import static org.firstinspires.ftc.teamcode.subsystems.Robot.rb;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

import org.firstinspires.ftc.teamcode.subsystems.actions.Wait;

public class DepositCondense implements Action {
    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        rb.claw.arm.down();
        rb.claw.hand.close();
        rb.claw.elbow.vertical();
        rb.depositSlide.retract();
        while (depositSlide.isBusy()) {
            new Wait(10);
        }
        return false;
    }
}

