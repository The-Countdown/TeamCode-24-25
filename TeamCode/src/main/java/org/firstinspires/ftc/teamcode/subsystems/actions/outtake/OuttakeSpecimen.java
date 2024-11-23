package org.firstinspires.ftc.teamcode.subsystems.actions.outtake;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

import org.firstinspires.ftc.teamcode.subsystems.DepositSlide;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

public class OuttakeSpecimen implements Action {
    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        Robot.rb.intake.wrist.horizontal();
        Robot.rb.intake.hand.open();
        Robot.rb.intakeSlide.retract();
        Robot.rb.intake.arm.rest();
        Robot.rb.intake.elbow.rest();
        Robot.rb.depositSlide.retract();
        Robot.rb.outtake.arm.upClip();
        Robot.rb.outtake.wrist.horizontal();
        Robot.rb.outtake.hand.open();
        return false;
    }
}

