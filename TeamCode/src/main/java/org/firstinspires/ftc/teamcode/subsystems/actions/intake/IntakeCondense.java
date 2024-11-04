package org.firstinspires.ftc.teamcode.subsystems.actions.intake;

import static org.firstinspires.ftc.teamcode.subsystems.Robot.HardwareDevices.intakeSlideL;
import static org.firstinspires.ftc.teamcode.subsystems.Robot.HardwareDevices.intakeSlideR;
import static org.firstinspires.ftc.teamcode.subsystems.Robot.rb;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

import org.firstinspires.ftc.teamcode.subsystems.actions.Wait;

public class IntakeCondense implements Action {
    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        rb.claw.arm.down();
        rb.claw.hand.close();
        rb.intake.up();
        new Wait(750);
        rb.intakeSlide.retract();
        while (intakeSlideL.isBusy() || intakeSlideR.isBusy()) {
            new Wait(10);
        }
        rb.intake.down();
        new Wait(500);
        rb.claw.hand.close();
        new Wait(100);
        rb.claw.arm.forwards();
        rb.intake.spinStop();
        return false;
    }
}

