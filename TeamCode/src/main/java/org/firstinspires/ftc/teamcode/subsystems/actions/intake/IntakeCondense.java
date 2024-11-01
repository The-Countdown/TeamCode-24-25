package org.firstinspires.ftc.teamcode.subsystems.actions.intake;

import static org.firstinspires.ftc.teamcode.subsystems.Robot.HardwareDevices.claw;
import static org.firstinspires.ftc.teamcode.subsystems.Robot.HardwareDevices.clawArm;
import static org.firstinspires.ftc.teamcode.subsystems.Robot.HardwareDevices.depositSlide;
import static org.firstinspires.ftc.teamcode.subsystems.Robot.HardwareDevices.intakePitchL;
import static org.firstinspires.ftc.teamcode.subsystems.Robot.HardwareDevices.intakePitchR;
import static org.firstinspires.ftc.teamcode.subsystems.Robot.HardwareDevices.intakeSlideL;
import static org.firstinspires.ftc.teamcode.subsystems.Robot.HardwareDevices.intakeSlideR;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.DepositSlide;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSlide;

public class IntakeCondense implements Action {
    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        try {
            clawArm.setPosition(Claw.ClawPosition.down);
            claw.setPosition(Claw.ClawPosition.open);
            intakePitchL.setPosition(Intake.IntakePosition.upL);
            intakePitchR.setPosition(Intake.IntakePosition.upR);
            Thread.sleep(750);
            intakeSlideL.setTargetPositionTolerance(IntakeSlide.IntakeSlidePosition.tolerance);
            intakeSlideR.setTargetPositionTolerance(IntakeSlide.IntakeSlidePosition.tolerance);
            intakeSlideL.setTargetPosition(IntakeSlide.IntakeSlidePosition.retracted);
            intakeSlideR.setTargetPosition(IntakeSlide.IntakeSlidePosition.retracted);
            intakeSlideL.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            intakeSlideR.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            intakeSlideL.setPower(IntakeSlide.IntakeSlidePower.move);
            intakeSlideR.setPower(IntakeSlide.IntakeSlidePower.move);
            while (!(((intakeSlideL.getCurrentPosition() + intakeSlideR.getCurrentPosition()) / 2) < 200)) {
                Thread.sleep(10);
            }
            intakePitchL.setPosition(Intake.IntakePosition.downL - 0.04);
            intakePitchR.setPosition(Intake.IntakePosition.downR + 0.04);
            Thread.sleep(500);
            intakePitchL.setPosition(Intake.IntakePosition.downL - 0.005);
            intakePitchR.setPosition(Intake.IntakePosition.downR + 0.005);
            Thread.sleep(500);
            claw.setPosition(Claw.ClawPosition.closed);
            Thread.sleep(500);
            clawArm.setPosition(Claw.ClawPosition.forwards);
            depositSlide.setTargetPositionTolerance(3);
            depositSlide.setTargetPosition(20);
            depositSlide.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            depositSlide.setPower(DepositSlide.DepositSlidePower.move);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        return false;
    }
}

