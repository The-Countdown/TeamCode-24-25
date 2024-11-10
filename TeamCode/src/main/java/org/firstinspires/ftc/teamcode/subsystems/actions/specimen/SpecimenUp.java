package org.firstinspires.ftc.teamcode.subsystems.actions.specimen;

import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

public class SpecimenUp implements Action {
    private final Robot robot;

    public SpecimenUp(Robot robot) {
        this.robot = robot;
    }

    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        robot.depositSlide.specimenHang();
        return false;
    }
}
