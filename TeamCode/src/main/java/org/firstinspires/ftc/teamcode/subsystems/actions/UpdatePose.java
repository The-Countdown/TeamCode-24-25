package org.firstinspires.ftc.teamcode.subsystems.actions;

import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

public class UpdatePose implements Action {
    private final Robot robot;
    private final LinearOpMode opMode;

    public UpdatePose(Robot robot, LinearOpMode opMode) {
        this.robot = robot;
        this.opMode = opMode;
    }

    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        while (!opMode.opModeIsActive()) {
            robot.updatePose();
        }
        return false;
    }
}
