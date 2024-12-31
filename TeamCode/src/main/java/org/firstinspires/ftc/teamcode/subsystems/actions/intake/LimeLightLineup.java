package org.firstinspires.ftc.teamcode.subsystems.actions.intake;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystems.Robot;

public class LimeLightLineup implements Action {
    private final Robot robot;
    public LimeLightLineup(Robot robot) {
        this.robot = robot;
    }
    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        Robot.HardwareDevices.flashLight.enableLed(true);
        while (robot.limeLight.goToLimelightPos(0.1, 0.1, 2.5)) {
            robot.limeLight.goToLimelightPos(0.1, 0.1, 2.5);
        }
        robot.drive.movePower(0, 0, 0);
        robot.intakeSlide.moveTo(Robot.rb.intakeSlide.avg() + 100);
        Robot.HardwareDevices.flashLight.enableLed(false);
        return false;
    }
}
