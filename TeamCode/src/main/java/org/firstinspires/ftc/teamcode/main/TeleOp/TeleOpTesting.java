package org.firstinspires.ftc.teamcode.main.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.R;
import org.firstinspires.ftc.teamcode.main.Auto.RoadRunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.DepositSlide;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSlide;
import org.firstinspires.ftc.teamcode.subsystems.Outtake;
import org.firstinspires.ftc.teamcode.subsystems.Robot;
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOpTesting")
@Config
public class TeleOpTesting extends LinearOpMode {
    public static double intakeYawThreshold = 0.1;
    public static double intakeYawMulti = 0.001;

    public static double yStickLMulti = 0.4;
    public static double xStickLMulti = 0.6;
    public static double xStickRMulti = 0.4;
    public boolean driveToggle = false;
    boolean depositMagnetPressed = false;
    boolean wasRightTriggerPressed = false;
    boolean wasLeftTriggerPressed = false;
    boolean toggleState = false;
    YawPitchRollAngles robotOrientation;

    @Override
    public void runOpMode() {
        Robot robot = new Robot(this);

        FtcDashboard dashboard = FtcDashboard.getInstance();

        waitForStart();

        TestingThread testingRunnable = new TestingThread(this, robot);
        Thread testingThread = new Thread(testingRunnable);
        testingThread.start();

        DriveThread driveRunnable = new DriveThread(this, robot);
        Thread driveThread = new Thread(driveRunnable);
        driveThread.start();

        while (opModeIsActive()) {
            robotOrientation = Robot.HardwareDevices.imu.getRobotYawPitchRollAngles();
            robot.roadRunner.updatePoseEstimate();

            int intakeAvg = ((Robot.HardwareDevices.intakeSlideL.getCurrentPosition() + Robot.HardwareDevices.intakeSlideR.getCurrentPosition()) / 2);

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
            TelemetryPacket packet = new TelemetryPacket();
            packet.put("Heading (deg)", Math.toDegrees(robot.roadRunner.pose.heading.real) - 57.2958);
            packet.put("PoseX", robot.roadRunner.pose.position.x);
            packet.put("PoseY", robot.roadRunner.pose.position.y);
            packet.put("Deposit Height", Robot.HardwareDevices.depositSlide.getCurrentPosition());
            packet.put("Intake Height Avg", (intakeAvg));
            packet.put("IntakeL Height", (Robot.HardwareDevices.intakeSlideL.getCurrentPosition()));
            packet.put("IntakeR Height", (Robot.HardwareDevices.intakeSlideR.getCurrentPosition()));
            packet.put("Outtake Arm", (Robot.HardwareDevices.depositClawArmBottom.getPosition()));
            dashboard.sendTelemetryPacket(packet);

            telemetry.addData("Heading", Math.toDegrees(robot.roadRunner.pose.heading.real) - 57.2958);
            telemetry.addData("PoseX", (robot.roadRunner.pose.position.x));
            telemetry.addData("PoseY", (robot.roadRunner.pose.position.y));
            telemetry.addLine();
            telemetry.addData("Deposit Height", Robot.HardwareDevices.depositSlide.getCurrentPosition());
            telemetry.addLine();
            telemetry.addData("IntakeL Height", (Robot.HardwareDevices.intakeSlideL.getCurrentPosition()));
            telemetry.addData("IntakeR Height", (Robot.HardwareDevices.intakeSlideR.getCurrentPosition()));
            telemetry.addLine();
            telemetry.addData("Deposit Magnet", Robot.HardwareDevices.depositMagnet.getValue());
            telemetry.addData("Deposit Magnet", Robot.HardwareDevices.depositMagnet.isPressed());
            telemetry.addLine();
            telemetry.addData("Angle", robotOrientation.getYaw(AngleUnit.DEGREES));
            telemetry.addData("Joystick Angle", Math.toDegrees(DriveThread.joystickAngle));
            telemetry.addData("Corrected Angle", Math.toDegrees(DriveThread.correctedAngle));
            telemetry.update();
            //endregion
        }
        robot = null;
    }
}
