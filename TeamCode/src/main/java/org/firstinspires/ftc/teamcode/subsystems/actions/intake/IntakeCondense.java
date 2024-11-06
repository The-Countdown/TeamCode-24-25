package org.firstinspires.ftc.teamcode.subsystems.actions.intake;

import static org.firstinspires.ftc.teamcode.subsystems.Robot.HardwareDevices.intakeSlideL;
import static org.firstinspires.ftc.teamcode.subsystems.Robot.HardwareDevices.intakeSlideR;
import static org.firstinspires.ftc.teamcode.subsystems.Robot.rb;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

public class IntakeCondense implements Action {
    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        rb.depositClaw.arm.down();
        rb.depositClaw.hand.close();
        rb.intakeClaw.up();
        if (!rb.safeSleep(750)) {
            return true;
        }
        rb.intakeSlide.retract();
        while (intakeSlideL.isBusy() || intakeSlideR.isBusy()) {
            if (!rb.safeSleep(10)) {
                return true;
            }
        }
        rb.intakeClaw.down();
        if (!rb.safeSleep(500)) {
            return true;
        }
        rb.depositClaw.hand.close();
        if (!rb.safeSleep(100)) {
            return true;
        }
        rb.depositClaw.arm.forwards();
        rb.intakeClaw.spinStop();
        return false;
    }
}

