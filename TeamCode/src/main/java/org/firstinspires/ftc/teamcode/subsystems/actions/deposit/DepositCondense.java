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

public class DepositCondense implements Action {
    private boolean initialized = false;

    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        if (!initialized) {
            try {
                if (depositSlide.getCurrentPosition() <= 700) {
                    depositSlide.setTargetPositionTolerance(DepositSlide.DepositSlidePosition.tolerance);
                    depositSlide.setTargetPosition(700);
                    depositSlide.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                    depositSlide.setPower(DepositSlide.DepositSlidePower.move);
                    while (!(depositSlide.getCurrentPosition() > (700 - DepositSlide.DepositSlidePosition.stepRange))) {
                        Thread.sleep(10);
                    }
                }
                clawArm.setPosition(Claw.ClawPosition.down);
                clawAngle.setPosition(Claw.ClawPosition.vertical);
                claw.setPosition(Claw.ClawPosition.closed);
                Thread.sleep(1000);
                depositSlide.setTargetPositionTolerance(DepositSlide.DepositSlidePosition.tolerance);
                depositSlide.setTargetPosition(DepositSlide.DepositSlidePosition.retracted);
                depositSlide.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                depositSlide.setPower(DepositSlide.DepositSlidePower.move);
                while (!(depositSlide.getCurrentPosition() < (DepositSlide.DepositSlidePosition.retracted + DepositSlide.DepositSlidePosition.stepRange))) {
                    Thread.sleep(10);
                }
                claw.setPosition(Claw.ClawPosition.open);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
        }
        initialized = true;
        return initialized;
    }
}

