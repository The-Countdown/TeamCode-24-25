package org.firstinspires.ftc.teamcode.subsystems.actions.intake;

import static org.firstinspires.ftc.teamcode.subsystems.Robot.HardwareDevices.claw;
import static org.firstinspires.ftc.teamcode.subsystems.Robot.HardwareDevices.clawArm;
import static org.firstinspires.ftc.teamcode.subsystems.Robot.HardwareDevices.intakePitchL;
import static org.firstinspires.ftc.teamcode.subsystems.Robot.HardwareDevices.intakePitchR;
import static org.firstinspires.ftc.teamcode.subsystems.Robot.HardwareDevices.intakeRoller;
import static org.firstinspires.ftc.teamcode.subsystems.Robot.HardwareDevices.intakeSlideL;
import static org.firstinspires.ftc.teamcode.subsystems.Robot.HardwareDevices.intakeSlideR;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSlide;

public class IntakeGround implements Action {
    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        try {
            intakeSlideL.setTargetPositionTolerance(IntakeSlide.IntakeSlidePosition.tolerance);
            intakeSlideR.setTargetPositionTolerance(IntakeSlide.IntakeSlidePosition.tolerance);
            intakeSlideL.setTargetPosition(IntakeSlide.IntakeSlidePosition.ground);
            intakeSlideR.setTargetPosition(IntakeSlide.IntakeSlidePosition.ground);
            intakeSlideL.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            intakeSlideR.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            intakeSlideL.setPower(IntakeSlide.IntakeSlidePower.move);
            intakeSlideR.setPower(IntakeSlide.IntakeSlidePower.move);
            intakePitchL.setPosition(Intake.IntakePosition.downL);
            intakePitchR.setPosition(Intake.IntakePosition.downR);
            intakeRoller.setPower(Intake.IntakePower.spinIn);
            while (!(((intakeSlideL.getCurrentPosition() + intakeSlideR.getCurrentPosition()) / 2) > (IntakeSlide.IntakeSlidePosition.ground - IntakeSlide.IntakeSlidePosition.stepRange))) {
                Thread.sleep(10);
            }} catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        return false;
    }
}

