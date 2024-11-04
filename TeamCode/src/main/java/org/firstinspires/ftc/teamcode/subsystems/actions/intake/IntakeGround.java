package org.firstinspires.ftc.teamcode.subsystems.actions.intake;

import static org.firstinspires.ftc.teamcode.subsystems.Robot.HardwareDevices.intakeSlideL;
import static org.firstinspires.ftc.teamcode.subsystems.Robot.HardwareDevices.intakeSlideR;
import static org.firstinspires.ftc.teamcode.subsystems.Robot.rb;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

import org.firstinspires.ftc.teamcode.subsystems.actions.Wait;

public class IntakeGround implements Action {
    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        rb.intakeSlide.ground();
        rb.intake.spinIn();
        while (intakeSlideL.isBusy() || intakeSlideR.isBusy()) {
            new Wait(10);
        }
        new Wait(100);
        return false;
    }
}

