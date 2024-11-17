package org.firstinspires.ftc.teamcode.main.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.subsystems.DepositSlide;
import org.firstinspires.ftc.teamcode.subsystems.Robot;


public class DriveThread extends Robot.HardwareDevices implements Runnable {
    private final LinearOpMode opMode;
    private final Gamepad gamepad1;
    private final Robot robot;

    public DriveThread(LinearOpMode opMode, Robot robot) {
        this.opMode = opMode;
        this.gamepad1 = opMode.gamepad1;
        this.robot = robot;
    }

    @Override
    public void run() {
        double yStickLMulti = TeleOp.yStickLMulti;
        double xStickLMulti = TeleOp.xStickLMulti;
        double xStickRMulti = TeleOp.xStickRMulti;
        boolean wasSharePressed = false;
        boolean driveToggle = false;
        YawPitchRollAngles robotOrientation;

        while (opMode.opModeIsActive()) {
            robotOrientation = Robot.HardwareDevices.imu.getRobotYawPitchRollAngles();
            double imuYaw = -robotOrientation.getYaw(AngleUnit.DEGREES);
            if (imuYaw < 0) {
                imuYaw += 360;
            }

            double xStickR = gamepad1.right_stick_x * (xStickRMulti + (gamepad1.right_trigger * 0.45) - (gamepad1.left_trigger * 0.1));
            double xStickL = gamepad1.left_stick_x * (xStickLMulti + (gamepad1.right_trigger * 0.5) - (gamepad1.left_trigger * 0.2));
            double yStickL = gamepad1.left_stick_y * -(yStickLMulti + (gamepad1.right_trigger * 0.5) - (gamepad1.left_trigger * 0.2));

            double joystickAngle = Math.atan2(yStickL, xStickL);
            double magnitudeL = Math.hypot(xStickL, yStickL);
            double correctedAngle = Math.toDegrees(joystickAngle) - imuYaw;

            if (correctedAngle < 0) {
                correctedAngle += 360;
            }

            correctedAngle = Math.toRadians(correctedAngle);

            double newXStickL = (magnitudeL * Math.cos(correctedAngle)) * xStickLMulti;
            double newYStickL = (magnitudeL * Math.sin(correctedAngle)) * yStickLMulti;

            // Trigger driveToggle
            boolean isSharePressed = gamepad1.share;

            if (isSharePressed && !wasSharePressed) {
                driveToggle = !driveToggle;
            }
            wasSharePressed = isSharePressed;

            if (driveToggle) {
                // Field Drive
                Robot.HardwareDevices.leftFront.setPower(newYStickL + newXStickL + xStickR);
                Robot.HardwareDevices.leftBack.setPower(newYStickL - newXStickL + xStickR);
                Robot.HardwareDevices.rightFront.setPower(newYStickL - newXStickL - xStickR);
                Robot.HardwareDevices.rightBack.setPower(newYStickL + newXStickL - xStickR);
            } else {
                // Normal Drive
                Robot.HardwareDevices.leftFront.setPower(yStickL + xStickL + xStickR);
                Robot.HardwareDevices.leftBack.setPower(yStickL - xStickL + xStickR);
                Robot.HardwareDevices.rightFront.setPower(yStickL - xStickL - xStickR);
                Robot.HardwareDevices.rightBack.setPower(yStickL + xStickL - xStickR);
            }
        }
    }
}