package org.firstinspires.ftc.teamcode.subsystems.actions.specimen;

import static org.firstinspires.ftc.teamcode.subsystems.Robot.HardwareDevices.depositSlide;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.subsystems.DepositSlide;

public class SpecimenDown implements Action {
    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        try {
            depositSlide.setTargetPositionTolerance(DepositSlide.DepositSlidePosition.tolerance);
            depositSlide.setTargetPosition(DepositSlide.DepositSlidePosition.specimenBar - 250);
            depositSlide.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            depositSlide.setPower(DepositSlide.DepositSlidePower.move);
            while (!(depositSlide.getCurrentPosition() < ((DepositSlide.DepositSlidePosition.specimenBar - 250) + DepositSlide.DepositSlidePosition.tolerance))) {
                Thread.sleep(10);
            }
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        return true;
    }
}
