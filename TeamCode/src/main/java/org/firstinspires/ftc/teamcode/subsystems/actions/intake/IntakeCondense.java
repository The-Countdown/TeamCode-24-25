package org.firstinspires.ftc.teamcode.subsystems.actions.intake;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

import org.firstinspires.ftc.teamcode.subsystems.Robot;

public class IntakeCondense implements Action {
    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        Robot.rb.outtake.arm.transfer();
        Robot.rb.outtake.hand.close();
        Robot.rb.intake.up();
        if (!Robot.rb.safeSleep(750)) {
            return true;
        }
        Robot.rb.intakeSlide.retract();
        while (Robot.HardwareDevices.intakeSlideL.isBusy() || Robot.HardwareDevices.intakeSlideR.isBusy()) {
            if (!Robot.rb.safeSleep(10)) {
                return true;
            }
        }
        Robot.rb.intake.down();
        if (!Robot.rb.safeSleep(500)) {
            return true;
        }
        Robot.rb.outtake.hand.close();
        if (!Robot.rb.safeSleep(100)) {
            return true;
        }
        return false;
    }
}

