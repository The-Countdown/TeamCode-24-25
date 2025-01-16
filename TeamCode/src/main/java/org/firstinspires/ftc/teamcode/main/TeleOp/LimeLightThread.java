package org.firstinspires.ftc.teamcode.main.TeleOp;

import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.main.Auto.RoadRunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
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
        boolean protection = false;
        robot.intake.arm.up();
        robot.intake.elbow.down();
        while (opMode.opModeIsActive()) {
            if (!gamepad1.a) {
                protection = false;
                continue;
            }
            if (protection) {
                continue;
            }
            protection = true;

            robot.intake.wrist.horizontal();

            if (!robot.safeSleep(100)) {
                continue;
            }

            double orientation = 0;
            orientation = Math.abs(robot.limeLight.getBlockOrientation());

            if (orientation != 0) {
                orientation /= 355;
                opMode.telemetry.addData("target servo position", Intake.IntakePosition.wristHorizontal - orientation);
                robot.roadRunner.updatePoseEstimate();
                Actions.runBlocking(new SequentialAction(
                        robot.limeLight.goToLimelightPos(0, 3, 2.5).build()
                ));
                if (!robot.safeSleep(100)) {
                    continue;
                }
                Robot.HardwareDevices.intakeClawAngle.setPosition(Intake.IntakePosition.wristHorizontal - orientation);
                if (!robot.safeSleep(200)) {
                    continue;
                }
                robot.intake.hand.open();
                if (!robot.safeSleep(300)) {
                    continue;
                }
                robot.intake.arm.down();
                if (!robot.safeSleep(300)) {
                    continue;
                }
                robot.intake.hand.close();
                if (!robot.safeSleep(300)) {
                    continue;
                }
                robot.intake.arm.up();
                robot.intake.wrist.horizontal();
                robot.intakeSlide.retract();
            }
            opMode.telemetry.update();
        }
    }
}
