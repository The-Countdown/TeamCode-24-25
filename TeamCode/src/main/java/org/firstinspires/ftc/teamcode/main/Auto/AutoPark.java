package org.firstinspires.ftc.teamcode.main.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystems.Robot;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "AutoPark")
public class AutoPark extends LinearOpMode {

    @Override
    public void runOpMode() {
        Robot robot = new Robot(this);
        waitForStart();

        if (opModeIsActive()) {
        }
    }
}
