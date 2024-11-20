package org.firstinspires.ftc.teamcode.subsystems.actions.outtake;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

import org.firstinspires.ftc.teamcode.subsystems.Robot;

public class OuttakeCondense implements Action {
    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        Robot.rb.outtake.arm.rest();
        Robot.rb.outtake.hand.close();
        Robot.rb.outtake.wrist.vertical();
        if (!Robot.rb.safeSleep(300)) {
            return true;
        }
        Robot.rb.depositSlide.retract();
        while (Robot.HardwareDevices.depositSlide.isBusy()) {
            if (!Robot.rb.safeSleep(10)) {
                return true;
            }
        }
        return false;
    }
}

