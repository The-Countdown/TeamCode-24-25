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

    @Override
    public void runOpMode() {
        Robot robot = new Robot(this);

        Robot.HardwareDevices.leftFront.setDirection(DcMotorEx.Direction.REVERSE);
        Robot.HardwareDevices.leftBack.setDirection(DcMotorEx.Direction.REVERSE);
        Robot.HardwareDevices.intakeSlideL.setDirection(DcMotorEx.Direction.REVERSE);
        Robot.HardwareDevices.depositSlide.setDirection(DcMotorEx.Direction.REVERSE);
        Robot.HardwareDevices.arm.setDirection(DcMotorEx.Direction.REVERSE);

        // Motor Modes and Settings
        Robot.HardwareDevices.leftFront.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        Robot.HardwareDevices.rightFront.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        Robot.HardwareDevices.leftBack.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        Robot.HardwareDevices.rightBack.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        YawPitchRollAngles robotOrientation;

        FtcDashboard dashboard = FtcDashboard.getInstance();

//        robot.intake.rest(); //illegal
//        robot.outtake.rest(); //illegal

        waitForStart();

        TestingThread testingRunnable = new TestingThread(this, robot);
        Thread testingThread = new Thread(testingRunnable);
        testingThread.start();

        DriveThread driveRunnable = new DriveThread(this, robot);
        Thread driveThread = new Thread(driveRunnable);
        driveThread.start();

        while (opModeIsActive()) {
            robot.updatePose();

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
        robot = null;
    }
}
