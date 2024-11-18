package org.firstinspires.ftc.teamcode.subsystems.actions.outtake;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

import org.firstinspires.ftc.teamcode.subsystems.Robot;

public class OuttakeHighNet implements Action {
    private final Robot robot;

    public OuttakeHighNet(Robot robot) {
        this.robot = robot;
    }

    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        robot.outtake.hand.close();
        robot.depositSlide.highBasket();
        if (!robot.safeSleep(300)) {
            return true;
        }
        robot.outtake.arm.back();
        robot.outtake.wrist.horizontal();
        while (Robot.HardwareDevices.depositSlide.isBusy()) {
            if (!robot.safeSleep(10)) {
                return true;
            }
        }
        return false;
    }
}

