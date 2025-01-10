package org.firstinspires.ftc.teamcode.subsystems.actions.intake;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.main.Auto.RoadRunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

public class LimeLightLineup implements Action {
    private final Robot robot;
    public LimeLightLineup(Robot robot) {
        this.robot = robot;
    }
    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        TrajectoryActionBuilder trag;
        int cycles = 0;
        trag = robot.limeLight.goToLimelightPos(0.1, 0.1, 2.5);
        while (trag == null) {
            trag = robot.limeLight.goToLimelightPos(0.1, 0.1, 2.5);
            cycles += 1;
            if (cycles >= 50000000) {
                return false;
            }
        }
        MecanumDrive.PARAMS.timeout = 0.2;
        Actions.runBlocking(new SequentialAction(
                trag.build()
        ));
        MecanumDrive.PARAMS.timeout = 0.1;
        return false;
    }
}
