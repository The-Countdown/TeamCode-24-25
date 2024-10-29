package org.firstinspires.ftc.teamcode.main.Auto;

import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystems.DepositActionHigh;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

@Autonomous
public class AutoTest extends LinearOpMode {

    @Override
    public void runOpMode() {
        Robot robot = new Robot(hardwareMap, telemetry, this);

        waitForStart();

        Actions.runBlocking(new DepositActionHigh());
    }
}
