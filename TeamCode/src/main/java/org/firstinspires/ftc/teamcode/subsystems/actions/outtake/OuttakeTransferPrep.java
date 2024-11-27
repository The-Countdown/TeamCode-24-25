package org.firstinspires.ftc.teamcode.subsystems.actions.outtake;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

import org.firstinspires.ftc.teamcode.subsystems.DepositSlide;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

public class OuttakeTransferPrep implements Action {
    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        Robot.rb.outtake.hand.open();
        Robot.rb.outtake.arm.forward();
        if (!Robot.rb.safeSleep(200)) {
            return true;
        }
        Robot.rb.depositSlide.transfer();
        return false;
    }
}

