package org.firstinspires.ftc.teamcode.subsystems.actions.intake;

import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

import org.firstinspires.ftc.teamcode.subsystems.DepositSlide;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

public class IntakePickUp implements Action {
    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        Robot.rb.intake.arm.down();
        if (!Robot.rb.safeSleep(250)) {
            return true;
        }
        Robot.rb.intake.hand.halfOpen();
        if (!Robot.rb.safeSleep(250)) {
            return true;
        }
        Robot.rb.intake.arm.up();
        return false;
    }
}
