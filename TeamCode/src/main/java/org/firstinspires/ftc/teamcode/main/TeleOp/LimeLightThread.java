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
                    pickUp();
                    robot.driveAvailable = true;
                } else {
                    robot.intake.hand.open();
                }
            }
            wasRightBumperPressed = isRightBumperPressed;
        }
    }

    void pickUp() {

        if (Robot.HardwareDevices.intakeClawAngle.getPosition() < 0.85) {
            robot.intake.wrist.horizontal();
            if (!robot.safeSleep(300)) {
                return;
            }
        }

        double orientation = 0;
        orientation = robot.limeLight.getBlockOrientation();

        if (orientation != 0) {
            orientation /= 355;
            opMode.telemetry.addData("target servo position", Intake.IntakePosition.wristHorizontal - orientation);

            double height = 6.3; // height of the camera in inches
            double moveAmountY = robot.limeLight.getLimeLightResult().getTy();
            double yDistance = height * Math.tan(Math.toRadians(moveAmountY));

            int target = (int)(Robot.HardwareDevices.intakeSlideL.getCurrentPosition() + 100 + (yDistance*90));


            if (target > IntakeSlide.IntakeSlidePosition.maximum) {
                target = IntakeSlide.IntakeSlidePosition.maximum;
            }

            if (target < IntakeSlide.IntakeSlidePosition.minimum) {
                target = IntakeSlide.IntakeSlidePosition.minimum;
            }

            robot.intakeSlide.moveTo(target);

            robot.roadRunner.updatePoseEstimate();
            Actions.runBlocking(new SequentialAction(
                    robot.limeLight.goToLimelightPos(0, -10, 2.5).build()
            ));

            double newOrientation = robot.limeLight.getBlockOrientation();
            if (newOrientation != 0) {
                orientation = newOrientation / 355;
            }
            
            Robot.HardwareDevices.intakeClawAngle.setPosition(Intake.IntakePosition.wristHorizontal - orientation);
            robot.intake.hand.open();
            if (!robot.safeSleep(300)) {
                return;
            }

            while (Robot.HardwareDevices.intakePitchL.getPosition() > Intake.IntakePosition.armDown) {
                Robot.HardwareDevices.intakePitchL.setPosition(Robot.HardwareDevices.intakePitchL.getPosition() - 0.01);
                Robot.HardwareDevices.intakePitchR.setPosition(Robot.HardwareDevices.intakePitchR.getPosition() - 0.01);
                if (!robot.safeSleep(5)) {
                    return;
                }
            }

            robot.intake.hand.close();
            if (!robot.safeSleep(300)) {
                return;
            }
            robot.intake.arm.up();
            robot.intake.wrist.horizontal();
            robot.intakeSlide.retract();
        }
        opMode.telemetry.update();
    }
}
