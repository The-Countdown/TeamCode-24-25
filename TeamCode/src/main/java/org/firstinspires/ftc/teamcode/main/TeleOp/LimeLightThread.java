package org.firstinspires.ftc.teamcode.main.TeleOp;

import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.main.Auto.RoadRunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSlide;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

public class LimeLightThread extends Robot.HardwareDevices implements Runnable {

    private final LinearOpMode opMode;
    private final Robot robot;
    private final Gamepad gamepad1;
    private final Gamepad gamepad2;
    public LimeLightThread(LinearOpMode opMode, Robot robot) {
        this.opMode = opMode;
        this.robot = robot;
        this.gamepad1 = opMode.gamepad1;
        this.gamepad2 = opMode.gamepad2;

    }
    @Override
    public void run() {
        //boolean protection = false;
        //robot.intake.arm.up();
        //robot.intake.elbow.down();
        boolean wasRightBumperPressed = false;
        boolean toggleStateRB = false;
        while (opMode.opModeIsActive()) {
//            if (!gamepad1.a) {
//                protection = false;
//                continue;
//            }
//            if (protection) {
//                continue;
//            }
//            protection = true;

            boolean isRightBumperPressed = gamepad2.right_bumper;

            if (isRightBumperPressed && !wasRightBumperPressed) {
                toggleStateRB = !toggleStateRB;

                if (toggleStateRB) {
                    robot.driveAvailable = false;
                    try {
                        robot.limeLight.pickUp();
                    } catch (Exception e) {
                        opMode.telemetry.addData("Pick Up Error", e.getMessage());
                    }
                    robot.driveAvailable = true;
                } else {
                    robot.intake.hand.open();
                }
            }
            wasRightBumperPressed = isRightBumperPressed;
        }
    }
}
