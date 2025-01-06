package org.firstinspires.ftc.teamcode.subsystems.actions.outtake;

import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

import org.firstinspires.ftc.teamcode.subsystems.DepositSlide;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

public class OuttakePreloadEsc implements Action {
    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        Robot.rb.depositSlide.specimenBar();
        while (!(Robot.HardwareDevices.depositSlide.getCurrentPosition() > 750));
        Robot.rb.outtake.arm.upLift();
        Robot.rb.outtake.wrist.horizontalFlip();
        return false;
    }
}
