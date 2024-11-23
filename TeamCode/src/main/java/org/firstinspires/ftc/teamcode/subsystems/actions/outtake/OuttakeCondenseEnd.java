package org.firstinspires.ftc.teamcode.subsystems.actions.outtake;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

import org.firstinspires.ftc.teamcode.subsystems.Robot;

public class OuttakeCondenseEnd implements Action {
    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        Robot.rb.depositSlide.specimenWall();
        while (Robot.HardwareDevices.depositSlide.isBusy()) {
            if (!Robot.rb.safeSleep(10)) {
                return true;
            }
        }
        Robot.rb.outtake.arm.rest();
        Robot.rb.outtake.wrist.vertical();
        Robot.rb.outtake.hand.close();
        if (!Robot.rb.safeSleep(500)) {
            return true;
        }
        Robot.rb.depositSlide.retract();
        return false;
    }
}

