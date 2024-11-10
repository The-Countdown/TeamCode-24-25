package org.firstinspires.ftc.teamcode.subsystems.actions.specimen;

import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

public class SpecimenDown implements Action {
    private final Robot robot;

    public SpecimenDown(Robot robot) {
        this.robot = robot;
    }

    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        robot.depositSlide.specimenWall();
        robot.outtake.hand.open();
        return false;
    }
}
