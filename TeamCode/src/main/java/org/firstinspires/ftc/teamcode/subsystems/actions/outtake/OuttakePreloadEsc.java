package org.firstinspires.ftc.teamcode.subsystems.actions.outtake;

import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

import org.firstinspires.ftc.teamcode.subsystems.DepositSlide;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

public class OuttakePreloadEsc implements Action {
    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        Robot.rb.depositSlide.specimenBarAlt();
        while (!(Robot.HardwareDevices.depositSlide.getCurrentPosition() > (DepositSlide.DepositSlidePosition.specimenBarAlt - 300))) {
            if (!Robot.rb.safeSleep(10)) {
                return true;
            }
        }
        Robot.rb.outtake.arm.upLift();
        Robot.rb.outtake.wrist.horizontal();
        return false;
    }
}
