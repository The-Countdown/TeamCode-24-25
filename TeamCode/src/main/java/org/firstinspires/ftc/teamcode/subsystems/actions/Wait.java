package org.firstinspires.ftc.teamcode.subsystems.actions;

import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

public class Wait implements Action {
    private final Robot robot;
    private final long duration;

    public Wait(Robot robot, long duration) {
        this.robot = robot;
        this.duration = duration;
    }

    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        return !robot.safeSleep(duration);
    }
}
