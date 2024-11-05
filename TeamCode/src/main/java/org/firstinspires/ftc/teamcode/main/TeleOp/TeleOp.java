package org.firstinspires.ftc.teamcode.main.TeleOp;

import static org.firstinspires.ftc.teamcode.subsystems.Robot.HardwareDevices.arm;
import static org.firstinspires.ftc.teamcode.subsystems.Robot.HardwareDevices.claw;
import static org.firstinspires.ftc.teamcode.subsystems.Robot.HardwareDevices.clawAngle;
import static org.firstinspires.ftc.teamcode.subsystems.Robot.HardwareDevices.clawArm;
import static org.firstinspires.ftc.teamcode.subsystems.Robot.HardwareDevices.depositMagnet;
import static org.firstinspires.ftc.teamcode.subsystems.Robot.HardwareDevices.depositSlide;
import static org.firstinspires.ftc.teamcode.subsystems.Robot.HardwareDevices.intakePitchL;
import static org.firstinspires.ftc.teamcode.subsystems.Robot.HardwareDevices.intakePitchR;
import static org.firstinspires.ftc.teamcode.subsystems.Robot.HardwareDevices.intakeSlideL;
import static org.firstinspires.ftc.teamcode.subsystems.Robot.HardwareDevices.intakeSlideR;
import static org.firstinspires.ftc.teamcode.subsystems.Robot.HardwareDevices.intakeYaw;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.DepositSlide;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSlide;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOp")
@Config
public class TeleOp extends LinearOpMode {
    public static double intakeYawThreshold = 0.1;
    public static double intakeYawMulti = 0.001;

    public static double yStickLMulti = 0.4;
    public static double xStickLMulti = 0.6;
    public static double xStickRMulti = 0.4;
    public boolean driveToggle = false;
    boolean depositMagnetPressed = false;
    @Override
    public void runOpMode() {
        Robot robot = new Robot(this);
        YawPitchRollAngles robotOrientation;

        FtcDashboard dashboard = FtcDashboard.getInstance();

        waitForStart();

        DepositThread depositRunnable = new DepositThread(this);
        Thread depositThread = new Thread(depositRunnable);
        depositThread.start();

        IntakeThread intakeRunnable = new IntakeThread(this);
        Thread intakeThread = new Thread(intakeRunnable);
        intakeThread.start();

        robot.claw.hand.open();
        robot.claw.arm.down();
        robot.claw.elbow.vertical();

        intakePitchL.setPosition(0.595);
        intakePitchR.setPosition(0.555);

        while (opModeIsActive()) {
            robot.drive.roadRunner.updatePoseEstimate();

            int yStickRInt  = (int) (gamepad2.right_stick_y * 30);
            int intakeAvg = (int) ((intakeSlideL.getCurrentPosition() + intakeSlideR.getCurrentPosition()) / 2);

            if (depositMagnet.isPressed()) {
                if (!depositMagnetPressed) {
                    Robot.HardwareDevices.depositSlide.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
                    Robot.HardwareDevices.depositSlide.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
                    Robot.HardwareDevices.depositSlide.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                    depositMagnetPressed = true;
                }
            } else {
                depositMagnetPressed = false;
            }

            //region Driving
            robotOrientation = Robot.HardwareDevices.imu.getRobotYawPitchRollAngles();
            double imuYaw = -robotOrientation.getYaw(AngleUnit.DEGREES);
            if (imuYaw < 0) {
                imuYaw += 360;
            }

            double xStickR = gamepad1.right_stick_x * (xStickRMulti + (gamepad1.right_trigger * 0.3) - (gamepad1.left_trigger * 0.1));
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
            double newYStickL = (magnitudeL * Math.sin(correctedAngle)) * -yStickLMulti;

            // Trigger driveToggle
            if (gamepad1.right_bumper) {
                driveToggle = true;
            }
            if (gamepad1.left_bumper) {
                driveToggle = false;
            }

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
            //endregion

            //region Subsystem Controls
            if (gamepad2.right_bumper) {
                robot.intake.spinIn();
            } else if (gamepad2.left_bumper) {
                robot.intake.spinOut();
            } else {
                robot.intake.spinStop();
            }

            if (gamepad2.right_stick_y != 0) {
                depositSlide.setTargetPosition(depositSlide.getTargetPosition() - yStickRInt);
            } else {
                depositSlide.setTargetPosition(depositSlide.getTargetPosition());
            }

            if (gamepad2.right_trigger > 0) {
                arm.setPower(-gamepad2.right_trigger);
            } else if (gamepad2.left_trigger > 0) {
                arm.setPower(gamepad2.left_trigger);
            } else
                robot.arm.stop();

            if (gamepad2.dpad_left) {
                robot.claw.hand.open();
            }

            if (gamepad1.square) {
                robot.claw.elbow.vertical();
            }
            if (gamepad1.triangle) {
                robot.claw.elbow.horizontal();
            }

            if (gamepad2.dpad_right && (depositSlide.getCurrentPosition() < 100)) {
                robot.claw.hand.open();
                robot.claw.arm.down();
            }

            if ((depositSlide.getTargetPosition() < DepositSlide.DepositSlidePosition.stopTolerance) &&
                    (depositSlide.getCurrentPosition() < DepositSlide.DepositSlidePosition.stopTolerance)) {
                robot.depositSlide.stop();
            }
            if ((!intakeSlideL.isBusy()) && (intakeSlideL.getTargetPosition() < IntakeSlide.IntakeSlidePosition.tolerance) &&
                    (intakeSlideL.getCurrentPosition() < IntakeSlide.IntakeSlidePosition.tolerance)) {
                intakeSlideL.setPower(IntakeSlide.IntakeSlidePower.stop);
            }
            if ((!intakeSlideR.isBusy()) && (intakeSlideR.getTargetPosition() < IntakeSlide.IntakeSlidePosition.tolerance) &&
                    (intakeSlideR.getCurrentPosition() < IntakeSlide.IntakeSlidePosition.tolerance)) {
                intakeSlideR.setPower(IntakeSlide.IntakeSlidePower.stop);
            }

            //endregion

            //region Telemetry
            TelemetryPacket packet = new TelemetryPacket();
            packet.put("Heading", Math.toDegrees(robot.drive.roadRunner.pose.heading.real));
            packet.put("PoseX", (robot.drive.roadRunner.pose.position.x));
            packet.put("PoseY", (robot.drive.roadRunner.pose.position.y));
            packet.put("Claw Position", claw.getPosition());
            packet.put("Claw Rotation", clawAngle.getPosition());
            packet.put("Deposit Height", depositSlide.getCurrentPosition());
            packet.put("Intake Height Avg", (intakeAvg));
            packet.put("IntakeL Height", (intakeSlideL.getCurrentPosition()));
            packet.put("IntakeR Height", (intakeSlideR.getCurrentPosition()));
            packet.put("Intake Yaw", intakeYaw.getPosition());
            packet.put("Intake Velocity", ((intakeSlideL.getVelocity() + intakeSlideR.getVelocity()) / 2));
            packet.put("IntakeL Velocity", intakeSlideL.getVelocity());
            packet.put("IntakeR Velocity", intakeSlideR.getVelocity());
            packet.put("New Y Stick L", newYStickL);
            packet.put("New X Stick L", newXStickL);
            packet.put("IMU Yaw", imuYaw);
            packet.put("Corrected Angle", correctedAngle);
            packet.put("Joystick Angle", joystickAngle);
            packet.put("Left Stick X", xStickL);
            packet.put("Left Stick Y", yStickL);
            packet.put("Touchpad X", gamepad2.touchpad_finger_1_x);
            packet.put("Touchpad Y", gamepad2.touchpad_finger_1_y);
            dashboard.sendTelemetryPacket(packet);

            telemetry.addData("Heading", Math.toDegrees(robot.drive.roadRunner.pose.heading.real));
            telemetry.addData("PoseX", (robot.drive.roadRunner.pose.position.x));
            telemetry.addData("PoseY", (robot.drive.roadRunner.pose.position.y));
            telemetry.addLine();
            telemetry.addData("Claw", claw.getPosition());
            telemetry.addData("Claw Rotation", clawAngle.getPosition());
            telemetry.addData("ClawArm", clawArm.getPosition());
            telemetry.addData("BackPos", Claw.Position.Arm.back);
            telemetry.addLine();
            telemetry.addData("Deposit Height", depositSlide.getCurrentPosition());
            telemetry.addLine();
            telemetry.addData("IntakeL Height", (intakeSlideL.getCurrentPosition()));
            telemetry.addData("IntakeR Height", (intakeSlideR.getCurrentPosition()));
            telemetry.addLine();
            telemetry.addData("IntakeL Velocity", intakeSlideL.getVelocity());
            telemetry.addData("IntakeR Velocity", intakeSlideR.getVelocity());
            telemetry.addLine();
            telemetry.addData("Intake Yaw", intakeYaw.getPosition());
            telemetry.addLine();
            telemetry.addData("newYStickL", newYStickL);
            telemetry.addData("newXStickL", newXStickL);
            telemetry.addLine();
            telemetry.addData("IMU Yaw", imuYaw);
            telemetry.addData("Corrected Angle", correctedAngle);
            telemetry.addData("Joystick Angle", joystickAngle);
            telemetry.addLine();
            telemetry.addData("Left Stick X", xStickL);
            telemetry.addData("Left Stick Y", yStickL);
            telemetry.addLine();
            telemetry.addData("Deposit Magnet", depositMagnet.getValue());
            telemetry.addData("Deposit Magnet", depositMagnet.isPressed());
            telemetry.update();
            //endregion
        }
    }
}