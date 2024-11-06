package org.firstinspires.ftc.teamcode.subsystems.actions;

import static org.firstinspires.ftc.teamcode.subsystems.Robot.rb;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.main.Auto.RoadRunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

public class UpdatePose implements Action {
    private final LinearOpMode opMode;


    public UpdatePose(LinearOpMode opMode) {
        this.opMode = opMode;
    }

    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        while (!opMode.opModeIsActive()) {
            rb.updatePose();
        }
        return false;
    }
}