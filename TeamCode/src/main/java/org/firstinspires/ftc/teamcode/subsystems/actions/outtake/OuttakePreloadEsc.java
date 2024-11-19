package org.firstinspires.ftc.teamcode.subsystems.actions.outtake;

import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

import org.firstinspires.ftc.teamcode.subsystems.DepositSlide;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

public class OuttakePreloadEsc implements Action {
    private final Robot robot;

    public OuttakePreloadEsc(Robot robot) {
        this.robot = robot;
    }

    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        robot.depositSlide.specimenBarAlt();
        while (!(Robot.HardwareDevices.depositSlide.getCurrentPosition() > (DepositSlide.DepositSlidePosition.specimenBarAlt - 300))) {
            if (!robot.safeSleep(10)) {
                return true;
            }
        }
        robot.outtake.arm.upLift();
        robot.outtake.wrist.horizontal();
        return false;
    }
}
