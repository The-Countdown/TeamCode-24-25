package org.firstinspires.ftc.teamcode.subsystems.actions.outtake;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

import org.firstinspires.ftc.teamcode.subsystems.DepositSlide;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

public class OuttakeActTwo implements Action {
    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        Robot.rb.intake.elbow.transfer();
        if (!Robot.rb.safeSleep(1000)) {
            return true;
        }
        Robot.rb.outtake.hand.close();
        if (!Robot.rb.safeSleep(250)) {
            return true;
        }
        Robot.rb.intake.hand.open();
        if (!Robot.rb.safeSleep(250)) {
            return true;
        }
        Robot.rb.depositSlide.highBasket();
        Robot.HardwareDevices.depositSlide.setPower(DepositSlide.DepositSlidePower.move/2);
        if (!Robot.rb.safeSleep(400)) {
            return true;
        }
        Robot.HardwareDevices.depositSlide.setPower(DepositSlide.DepositSlidePower.move);
        while (!(Robot.HardwareDevices.depositSlide.getCurrentPosition() > (DepositSlide.DepositSlidePosition.highBasket - 300))) {
            if (!Robot.rb.safeSleep(10)) {
                return true;
            }
        }
        Robot.rb.intake.arm.up();
        Robot.rb.intake.elbow.down();
        Robot.rb.outtake.arm.back();
        Robot.rb.outtake.wrist.horizontal();
        return false;
    }
}

