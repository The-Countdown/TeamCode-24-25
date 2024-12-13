package org.firstinspires.ftc.teamcode.subsystems.actions.outtake;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

import org.firstinspires.ftc.teamcode.subsystems.DepositSlide;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

public class OuttakeHighNet implements Action {
    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        Robot.rb.outtake.hand.close();
        Robot.rb.depositSlide.highBasket();
        while (!(Robot.HardwareDevices.depositSlide.getCurrentPosition() > (DepositSlide.DepositSlidePosition.highBasket - 1500))) {
            telemetryPacket.addLine("Waiting...");
        }
        telemetryPacket.clearLines();
        Robot.rb.outtake.arm.back();
        Robot.rb.outtake.wrist.horizontal();
        return false;
    }
}

