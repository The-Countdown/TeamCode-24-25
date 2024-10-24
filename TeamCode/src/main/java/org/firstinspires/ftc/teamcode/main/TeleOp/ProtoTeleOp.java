package org.firstinspires.ftc.teamcode.main.TeleOp;

import static org.firstinspires.ftc.teamcode.subsystems.Robot.HardwareDevices.claw;
import static org.firstinspires.ftc.teamcode.subsystems.Robot.HardwareDevices.clawAngle;
import static org.firstinspires.ftc.teamcode.subsystems.Robot.HardwareDevices.clawArm;
import static org.firstinspires.ftc.teamcode.subsystems.Robot.HardwareDevices.depositSlide;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.DepositSlide;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

import java.util.Locale;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "ProtoTeleOp", group = "TeleOp")
@Config
public class ProtoTeleOp extends LinearOpMode {
    public static double intakeYawThreshold = 0.1;
    public static double intakeYawMulti = 0.001;

    public static double yStickLMulti = -0.8;
    public static double xStickLMulti = 1.5;
    public static double xStickRMulti = 0.6;
    public boolean driveToggle = false;
    @Override
    public void runOpMode() {
        Robot robot = new Robot(hardwareMap, telemetry, this);

        YawPitchRollAngles robotOrientation;

        FtcDashboard dashboard = FtcDashboard.getInstance();

        waitForStart();

        DepositThread depositRunnable = new DepositThread("depSequences", hardwareMap, gamepad1, gamepad2, this, robot);
        Thread depositThread = new Thread(depositRunnable);
        depositThread.start();

        IntakeThread intakeRunnable = new IntakeThread("inSequences", hardwareMap, gamepad2, this, robot);
        Thread intakeThread = new Thread(intakeRunnable);
        intakeThread.start();

        //Pose2d beginPos = robot.drive.getRobotPos();

        claw.setPosition(Claw.ClawPosition.open);
        clawArm.setPosition(Claw.ClawPosition.down);
        clawAngle.setPosition(Claw.ClawPosition.vertical);

        while (opModeIsActive()) {

            //Pose2d robotPosition = robot.drive.getRobotPos();

            //double x = robotPosition.position.x;
            //double y = robotPosition.position.y;
            //double heading = robotPosition.heading.real;

            //region Driving
             robotOrientation = Robot.HardwareDevices.imu.getRobotYawPitchRollAngles();
            double imuYaw = -robotOrientation.getYaw(AngleUnit.DEGREES);
            if (imuYaw < 0) {
                imuYaw += 360;
            }

            double xStickR = gamepad1.right_stick_x * xStickRMulti;
            double xStickL = gamepad1.left_stick_x * xStickLMulti;
            double yStickL = gamepad1.left_stick_y * yStickLMulti;

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
            if (Math.abs(gamepad2.right_stick_x) > intakeYawThreshold) {
                robot.intake.yaw(Robot.HardwareDevices.intakeYaw.getPosition() - (gamepad2.right_stick_x * intakeYawMulti));
            }

            while (gamepad2.left_stick_button) {
                if (gamepad2.cross) {
                    robot.claw.vertical();
                } else if (gamepad2.circle) {
                    robot.claw.horizontal();
                }
            }
            while (gamepad2.share) {
                if (gamepad2.cross) {
                    robot.claw.down();
                } else if (gamepad2.circle) {
                    robot.claw.forwards();
                } else if (gamepad2.square) {
                    robot.claw.up();
                } else if (gamepad2.triangle) {
                    robot.claw.back();
                }
            }

            if (gamepad2.cross) {
                robot.intake.up();
            } else if (gamepad2.circle) {
                robot.intake.down();
            }
            if (gamepad2.square) {
                robot.claw.open();
            } else if (gamepad2.triangle) {
                robot.claw.close();
            }

            if (gamepad2.right_bumper) {
                robot.intake.spinIn();
            } else if (gamepad2.left_bumper) {
                robot.intake.spinOut();
            }

            if (gamepad1.right_trigger > 0) {
                Robot.HardwareDevices.arm.setPower(-gamepad1.right_trigger);
            } else if (gamepad1.left_trigger > 0) {
                Robot.HardwareDevices.arm.setPower(gamepad1.left_trigger);
            } else
                robot.arm.stop();

            if (gamepad1.dpad_up) {
                robot.intakeSlide.move(1000);
            } else if (gamepad1.dpad_down) {
                robot.intakeSlide.retract();
            }
            if (gamepad2.dpad_left) {
                robot.depositSlide.deposit();
            }
            if (gamepad1.circle) {
                claw.setPosition(Claw.ClawPosition.open);
            }

            if (gamepad2.dpad_right) {
                robot.depositSlide.condense();
            }

            if ((!depositSlide.isBusy()) && (depositSlide.getTargetPosition() < DepositSlide.DepositSlidePosition.stopTolerance)) {
                depositSlide.setPower(DepositSlide.DepositSlidePower.stop);
            }
            //endregion

            //region Telemetry
            TelemetryPacket packet = new TelemetryPacket();
            //packet.put("X Position", x);
            //packet.put("Y Position", y);
            //packet.put("Rotation", heading);
            packet.put("Claw Position", claw.getPosition());
            packet.put("Claw Rotation", clawAngle.getPosition());
            packet.put("Deposit Height", depositSlide.getCurrentPosition());
            packet.put("Intake Height", Robot.HardwareDevices.intakeSlide.getCurrentPosition());
            packet.put("Intake Yaw", Robot.HardwareDevices.intakeYaw.getPosition());
            packet.put("Intake Velocity", Robot.HardwareDevices.intakeSlide.getVelocity());
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

            //telemetry.addData("Position", String.format(Locale.US,"X: %.2f, Y: %.2f, Heading: %.2f", x, y, heading));
            telemetry.addLine();
            telemetry.addData("Claw", claw.getPosition());
            telemetry.addData("Claw Rotation", clawAngle.getPosition());
            telemetry.addLine();
            telemetry.addData("Deposit Height", depositSlide.getCurrentPosition());
            telemetry.addData("Intake Height", Robot.HardwareDevices.intakeSlide.getCurrentPosition());
            telemetry.addLine();
            telemetry.addData("Intake Yaw", Robot.HardwareDevices.intakeYaw.getPosition());
            telemetry.addData("Intake Velocity", Robot.HardwareDevices.intakeSlide.getVelocity());
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
            telemetry.update();
            //endregion
        }
    }
}
