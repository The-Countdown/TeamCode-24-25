package org.firstinspires.ftc.teamcode.subsystems.actions.outtake;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

import org.firstinspires.ftc.teamcode.subsystems.Robot;

public class OuttakeSpecimenAlt implements Action {
    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        Robot.rb.depositSlide.move(300);
        Robot.rb.outtake.arm.upLift();
        Robot.rb.outtake.wrist.horizontal();
        Robot.rb.outtake.hand.open();
        return false;
    }
}

