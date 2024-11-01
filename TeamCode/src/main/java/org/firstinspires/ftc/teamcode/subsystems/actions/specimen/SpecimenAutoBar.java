package org.firstinspires.ftc.teamcode.subsystems.actions.specimen;

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

public class SpecimenAutoBar implements Action {
    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        try {
            depositSlide.setTargetPositionTolerance(DepositSlide.DepositSlidePosition.tolerance);
            depositSlide.setTargetPosition(120);
            depositSlide.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            depositSlide.setPower(DepositSlide.DepositSlidePower.move);
            Thread.sleep(500);
            claw.setPosition(Claw.ClawPosition.closed);
            Thread.sleep(500);
            depositSlide.setTargetPositionTolerance(DepositSlide.DepositSlidePosition.tolerance);
            depositSlide.setTargetPosition(DepositSlide.DepositSlidePosition.specimenBar);
            depositSlide.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            depositSlide.setPower(DepositSlide.DepositSlidePower.move);
            Thread.sleep(750);
            clawArm.setPosition(Claw.ClawPosition.upLift);
            clawAngle.setPosition(Claw.ClawPosition.horizontal);
            while (!(depositSlide.getCurrentPosition() > (DepositSlide.DepositSlidePosition.specimenBar - DepositSlide.DepositSlidePosition.stepRange))) {
                Thread.sleep(10);
            }
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        return false;
    }
}
