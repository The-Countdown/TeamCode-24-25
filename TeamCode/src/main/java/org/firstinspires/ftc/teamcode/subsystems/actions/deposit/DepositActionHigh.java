package org.firstinspires.ftc.teamcode.subsystems.actions.deposit;

import static org.firstinspires.ftc.teamcode.subsystems.Robot.HardwareDevices.arm;
import static org.firstinspires.ftc.teamcode.subsystems.Robot.HardwareDevices.claw;
import static org.firstinspires.ftc.teamcode.subsystems.Robot.HardwareDevices.clawAngle;
import static org.firstinspires.ftc.teamcode.subsystems.Robot.HardwareDevices.clawArm;
import static org.firstinspires.ftc.teamcode.subsystems.Robot.HardwareDevices.depositSlide;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.DepositSlide;

public class DepositActionHigh implements Action {
    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        try {
            claw.setPosition(Claw.ClawPosition.closed);
            depositSlide.setTargetPositionTolerance(DepositSlide.DepositSlidePosition.tolerance);
            depositSlide.setTargetPosition(DepositSlide.DepositSlidePosition.highBasket);
            depositSlide.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            depositSlide.setPower(DepositSlide.DepositSlidePower.move);
            Thread.sleep(300);
            clawArm.setPosition(Claw.ClawPosition.back);
            clawAngle.setPosition(Claw.ClawPosition.horizontal);
            while (!(depositSlide.getCurrentPosition() > (DepositSlide.DepositSlidePosition.highBasket - DepositSlide.DepositSlidePosition.stepRange))) {
                Thread.sleep(10);
            }
            Thread.sleep(400);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        return false;
    }
}

