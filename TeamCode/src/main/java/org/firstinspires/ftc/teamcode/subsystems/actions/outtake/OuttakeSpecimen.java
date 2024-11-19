package org.firstinspires.ftc.teamcode.subsystems.actions.outtake;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

import org.firstinspires.ftc.teamcode.subsystems.DepositSlide;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

public class OuttakeSpecimen implements Action {
    private final Robot robot;

    public OuttakeSpecimen(Robot robot) {
        this.robot = robot;
    }

    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        robot.intake.wrist.horizontal();
        robot.intake.hand.open();
        robot.intakeSlide.retract();
        robot.intake.elbow.up();
        robot.depositSlide.specimenWall();
        if (!robot.safeSleep(600)) {
            return true;
        }
        robot.intake.arm.rest();
        robot.intake.elbow.rest();
        while (!(Robot.HardwareDevices.depositSlide.getCurrentPosition() > (DepositSlide.DepositSlidePosition.specimenWall - DepositSlide.DepositSlidePosition.stepRange))) {
            if (!robot.safeSleep(10)) {
                return true;
            }
        }
        robot.outtake.arm.upClip();
        robot.outtake.wrist.horizontal();
        if (!robot.safeSleep(750)) {
            return true;
        }
        robot.depositSlide.retract();
        Robot.HardwareDevices.depositSlide.setPower(DepositSlide.DepositSlidePower.move/2);
        robot.outtake.hand.open();
        return false;
    }
}

