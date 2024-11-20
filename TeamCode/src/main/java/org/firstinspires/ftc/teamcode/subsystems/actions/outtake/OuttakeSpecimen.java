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
        Robot.rb.intake.elbow.up();
        Robot.rb.depositSlide.specimenWall();
        if (!Robot.rb.safeSleep(600)) {
            return true;
        }
        Robot.rb.intake.arm.rest();
        Robot.rb.intake.elbow.rest();
        while (!(Robot.HardwareDevices.depositSlide.getCurrentPosition() > (DepositSlide.DepositSlidePosition.specimenWall - DepositSlide.DepositSlidePosition.stepRange))) {
            if (!Robot.rb.safeSleep(10)) {
                return true;
            }
        }
        Robot.rb.outtake.arm.upClip();
        Robot.rb.outtake.wrist.horizontal();
        if (!Robot.rb.safeSleep(750)) {
            return true;
        }
        Robot.rb.depositSlide.retract();
        Robot.HardwareDevices.depositSlide.setPower(DepositSlide.DepositSlidePower.move/2);
        Robot.rb.outtake.hand.open();
        return false;
    }
}

