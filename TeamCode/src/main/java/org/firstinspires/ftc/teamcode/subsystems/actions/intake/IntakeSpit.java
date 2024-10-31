package org.firstinspires.ftc.teamcode.subsystems.actions.intake;

import static org.firstinspires.ftc.teamcode.subsystems.Robot.HardwareDevices.intakeRoller;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

import org.firstinspires.ftc.teamcode.subsystems.Intake;

public class IntakeSpit implements Action {
    private boolean initialized = false;

    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        if (!initialized) {
            try {
                intakeRoller.setPower(Intake.IntakePower.spinOut);
                Thread.sleep(250);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
        }
        initialized = true;
        return initialized;
    }
}

