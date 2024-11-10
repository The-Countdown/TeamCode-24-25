package org.firstinspires.ftc.teamcode.subsystems.actions.deposit;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

import org.firstinspires.ftc.teamcode.subsystems.Robot;

public class DepositCondense implements Action {
    private final Robot robot;

    public DepositCondense(Robot robot) {
        this.robot = robot;
    }

    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        robot.outtake.arm.down();
        robot.outtake.hand.close();
        robot.outtake.wrist.vertical();
        robot.depositSlide.retract();
        while (Robot.HardwareDevices.depositSlide.isBusy()) {
            if (!robot.safeSleep(10)) {
                return true;
            }
        }
        return false;
    }
}

