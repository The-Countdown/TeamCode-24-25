package org.firstinspires.ftc.teamcode.main.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
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
        Robot robot = Robot.getInstance(this);

        YawPitchRollAngles robotOrientation;

        FtcDashboard dashboard = FtcDashboard.getInstance();

        robot.intake.rest(); //illegal
        robot.outtake.rest(); //illegal

        waitForStart();

        robot.intake.restEsc();
        sleep(600);
        robot.intake.elbow.down();

        DepositThread depositRunnable = new DepositThread(this, robot);
        Thread depositThread = new Thread(depositRunnable);
        depositThread.start();

        IntakeThread intakeRunnable = new IntakeThread(this, robot);
        Thread intakeThread = new Thread(intakeRunnable);
        intakeThread.start();

        while (opModeIsActive()) {
            robot.updatePose();

            int yStickRInt = (int) (gamepad2.right_stick_y * 30);
            int intakeAvg = (int) ((Robot.HardwareDevices.intakeSlideL.getCurrentPosition() + Robot.HardwareDevices.intakeSlideR.getCurrentPosition()) / 2);

            if (Robot.HardwareDevices.depositMagnet.isPressed()) {
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
            if ((!Robot.HardwareDevices.depositSlide.isBusy()) && (Robot.HardwareDevices.depositSlide.getTargetPosition() < DepositSlide.DepositSlidePosition.stopTolerance) &&
                    (Robot.HardwareDevices.depositSlide.getCurrentPosition() < DepositSlide.DepositSlidePosition.stopTolerance)) {
                Robot.HardwareDevices.depositSlide.setPower(DepositSlide.DepositSlidePower.stop);
            }
            if ((!Robot.HardwareDevices.intakeSlideL.isBusy()) && (Robot.HardwareDevices.intakeSlideL.getTargetPosition() < 10) &&
                    (Robot.HardwareDevices.intakeSlideL.getCurrentPosition() < 10)) {
                Robot.HardwareDevices.intakeSlideL.setPower(IntakeSlide.IntakeSlidePower.stop);
            }
            if ((!Robot.HardwareDevices.intakeSlideR.isBusy()) && (Robot.HardwareDevices.intakeSlideR.getTargetPosition() < 10) &&
                    (Robot.HardwareDevices.intakeSlideR.getCurrentPosition() < 10)) {
                Robot.HardwareDevices.intakeSlideR.setPower(IntakeSlide.IntakeSlidePower.stop);
            }
            //endregion

            //region Telemetry
            robot.updatePose();
            TelemetryPacket packet = new TelemetryPacket();
            packet.put("Heading", Math.toDegrees(robot.dreadDrive.pose.heading.real));
            packet.put("PoseX", (robot.dreadDrive.pose.position.x));
            packet.put("PoseY", (robot.dreadDrive.pose.position.y));
            packet.put("Deposit Height", Robot.HardwareDevices.depositSlide.getCurrentPosition());
            packet.put("Intake Height Avg", (intakeAvg));
            packet.put("IntakeL Height", (Robot.HardwareDevices.intakeSlideL.getCurrentPosition()));
            packet.put("IntakeR Height", (Robot.HardwareDevices.intakeSlideR.getCurrentPosition()));
            dashboard.sendTelemetryPacket(packet);

            telemetry.addData("Heading", Math.toDegrees(robot.dreadDrive.pose.heading.real));
            telemetry.addData("PoseX", (robot.dreadDrive.pose.position.x));
            telemetry.addData("PoseY", (robot.dreadDrive.pose.position.y));
            telemetry.addLine();
            telemetry.addData("Deposit Height", Robot.HardwareDevices.depositSlide.getCurrentPosition());
            telemetry.addLine();
            telemetry.addData("IntakeL Height", (Robot.HardwareDevices.intakeSlideL.getCurrentPosition()));
            telemetry.addData("IntakeR Height", (Robot.HardwareDevices.intakeSlideR.getCurrentPosition()));
            telemetry.addLine();
            telemetry.addData("Deposit Magnet", Robot.HardwareDevices.depositMagnet.getValue());
            telemetry.addData("Deposit Magnet", Robot.HardwareDevices.depositMagnet.isPressed());
            telemetry.update();
            //endregion
        }
    }
}
