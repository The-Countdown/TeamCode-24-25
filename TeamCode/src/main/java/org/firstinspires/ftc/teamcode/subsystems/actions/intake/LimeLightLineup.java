package org.firstinspires.ftc.teamcode.subsystems.actions.intake;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

import org.firstinspires.ftc.teamcode.subsystems.Robot;

public class LimeLightLineup implements Action {
    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        Robot.rb.limeLight.goToLimeLightPosLoop(0.1, 0.1, 2.5);
        Robot.rb.intakeSlide.moveTo(Robot.rb.intakeSlide.avg() + 25);
        while (Robot.HardwareDevices.intakeSlideL.isBusy());
        return false;
    }
}
