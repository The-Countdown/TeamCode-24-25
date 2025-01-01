package org.firstinspires.ftc.teamcode.subsystems.actions.intake;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystems.Robot;

public class LimeLightLineup implements Action {
    private final Robot robot;
    public LimeLightLineup(Robot robot) {
        this.robot = robot;
    }
    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        TrajectoryActionBuilder trag;
        trag = robot.limeLight.goToLimelightPos(0.1, 0.1, 2.5);
        while (trag == null) {
            trag = robot.limeLight.goToLimelightPos(0.1, 0.1, 2.5);
        }
        Actions.runBlocking(new SequentialAction(
                trag.build()
        ));
        robot.intakeSlide.moveTo(Robot.rb.intakeSlide.avg() + 100);
        return false;
    }
}
