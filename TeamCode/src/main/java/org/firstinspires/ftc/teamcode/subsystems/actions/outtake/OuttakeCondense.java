package org.firstinspires.ftc.teamcode.subsystems.actions.outtake;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

import org.firstinspires.ftc.teamcode.subsystems.Robot;

public class OuttakeCondense implements Action {
    private final Robot robot;

    public OuttakeCondense(Robot robot) {
        this.robot = robot;
    }

    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        robot.outtake.arm.transfer();
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

