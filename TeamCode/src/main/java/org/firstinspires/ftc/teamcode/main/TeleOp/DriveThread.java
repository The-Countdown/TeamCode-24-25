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

    public static double yStickLMulti;
    public static double xStickLMulti;
    public static double xStickRMulti;

    public static double joystickAngle;
    public static double magnitudeL;
    public static double correctedAngle;

    public static double imuYaw;

    @Override
    public void run() {
        yStickLMulti = TeleOp.yStickLMulti;
        xStickLMulti = TeleOp.xStickLMulti;
        xStickRMulti = TeleOp.xStickRMulti;
        boolean wasSharePressed = false;
        boolean driveToggle = false;
        YawPitchRollAngles robotOrientation;

        while (opMode.opModeIsActive()) {
            robotOrientation = Robot.HardwareDevices.imu.getRobotYawPitchRollAngles();
            imuYaw = -robotOrientation.getYaw(AngleUnit.DEGREES);
            if (imuYaw < 0) {
                imuYaw += 360;
            }

            double xStickR = gamepad1.right_stick_x * (xStickRMulti + (gamepad1.right_trigger * 0.45) - (gamepad1.left_trigger * 0.1));
            double xStickL = gamepad1.left_stick_x * (xStickLMulti + (gamepad1.right_trigger * 0.5) - (gamepad1.left_trigger * 0.2));
            double yStickL = gamepad1.left_stick_y * -(yStickLMulti + (gamepad1.right_trigger * 0.5) - (gamepad1.left_trigger * 0.2));

            joystickAngle = Math.atan2(xStickL, yStickL);
            magnitudeL = Math.hypot(xStickL, yStickL);
            correctedAngle = Math.toDegrees(joystickAngle) - imuYaw + Math.toRadians(-90);

            if (correctedAngle < 0) {
                correctedAngle += 360;
            }

            correctedAngle = Math.toRadians(correctedAngle);

            // does not work
//            double newXStickL = ((magnitudeL * 4) * Math.cos(correctedAngle)) * xStickLMulti;
//            double newYStickL = ((magnitudeL * 4) * Math.sin(correctedAngle)) * yStickLMulti;
            double newXStickL = ((magnitudeL * 4) * Math.sin(correctedAngle)) * xStickLMulti;
            double newYStickL = ((magnitudeL * 4) * Math.cos(correctedAngle)) * yStickLMulti;

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