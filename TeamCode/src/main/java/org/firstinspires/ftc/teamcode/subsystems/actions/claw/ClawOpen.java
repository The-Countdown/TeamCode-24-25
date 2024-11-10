package org.firstinspires.ftc.teamcode.subsystems.actions.claw;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

import org.firstinspires.ftc.teamcode.subsystems.Robot;

public class ClawOpen implements Action {
    private final Robot robot;

    public ClawOpen(Robot robot) {
        this.robot = robot;
    }

    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        robot.outtake.hand.open();
        return false;
    }
}

