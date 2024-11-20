package org.firstinspires.ftc.teamcode.subsystems.actions.outtake;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

import org.firstinspires.ftc.teamcode.subsystems.Robot;

public class OuttakeHighNet implements Action {
    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        Robot.rb.outtake.hand.close();
        Robot.rb.depositSlide.highBasket();
        if (!Robot.rb.safeSleep(300)) {
            return true;
        }
        Robot.rb.outtake.arm.back();
        Robot.rb.outtake.wrist.horizontal();
        while (Robot.HardwareDevices.depositSlide.isBusy()) {
            if (!Robot.rb.safeSleep(10)) {
                return true;
            }
        }
        return false;
    }
}

