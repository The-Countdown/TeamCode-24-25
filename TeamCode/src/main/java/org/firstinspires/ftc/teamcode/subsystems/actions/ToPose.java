package org.firstinspires.ftc.teamcode.subsystems.actions;

import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

public class ToPose implements Action {
    private final Robot robot;
    private final Pose2d pose;

    public ToPose(Robot robot, Pose2d pose) {
        this.robot = robot;
        this.pose = pose;
    }

    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        robot.drive.toPose(pose);
        return false;
    }
}
