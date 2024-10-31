package org.firstinspires.ftc.teamcode.subsystems.actions.deposit;

import static org.firstinspires.ftc.teamcode.subsystems.Robot.HardwareDevices.claw;
import static org.firstinspires.ftc.teamcode.subsystems.Robot.HardwareDevices.clawAngle;
import static org.firstinspires.ftc.teamcode.subsystems.Robot.HardwareDevices.clawArm;
import static org.firstinspires.ftc.teamcode.subsystems.Robot.HardwareDevices.depositSlide;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.DepositSlide;

public class DepositActionHigh implements Action {
    private boolean initialized = false;

    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        if (!initialized) {
            try {
                claw.setPosition(Claw.ClawPosition.closed);
                depositSlide.setTargetPositionTolerance(DepositSlide.DepositSlidePosition.tolerance);
                depositSlide.setTargetPosition(DepositSlide.DepositSlidePosition.highBasket);
                depositSlide.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                depositSlide.setPower(DepositSlide.DepositSlidePower.move);
                while (!(depositSlide.getCurrentPosition() > (DepositSlide.DepositSlidePosition.highBasket - DepositSlide.DepositSlidePosition.stepRange))) {
                    Thread.sleep(10);
                }
                clawArm.setPosition(Claw.ClawPosition.back);
                clawAngle.setPosition(Claw.ClawPosition.horizontal);
                Thread.sleep(1000);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
        }
        initialized = true;
        return initialized;
    }
}

