package org.firstinspires.ftc.teamcode.main.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.subsystems.DepositSlide;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSlide;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOp")
@Config
public class TeleOp extends LinearOpMode {
    public static double yStickLMulti = 0.4;
    public static double xStickLMulti = 0.6;
    public static double xStickRMulti = 0.3;
    public static boolean depositMagnetPressed = false;

    @Override
    public void runOpMode() {
        Robot robot = new Robot(this);

        FtcDashboard dashboard = FtcDashboard.getInstance();

        waitForStart();

        robot.intake.rest();

        DriveThread driveRunnable = new DriveThread(this, robot);
        Thread driveThread = new Thread(driveRunnable);
        driveThread.start();

        DepositThread depositRunnable = new DepositThread(this, robot);
        Thread depositThread = new Thread(depositRunnable);
        depositThread.start();

        IntakeThread intakeRunnable = new IntakeThread(this, robot);
        Thread intakeThread = new Thread(intakeRunnable);
        intakeThread.start();

        while (opModeIsActive()) {
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

            //region Safety
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

            if (gamepad1.guide || gamepad2.ps) {
                Robot.HardwareDevices.depositSlide.setPower(DepositSlide.DepositSlidePower.stop);
                Robot.HardwareDevices.intakeSlideL.setPower(IntakeSlide.IntakeSlidePower.stop);
                Robot.HardwareDevices.intakeSlideR.setPower(IntakeSlide.IntakeSlidePower.stop);
            }
            //endregion

            //region Telemetry
            robot.roadRunner.updatePoseEstimate();

            TelemetryPacket packet = new TelemetryPacket();
            packet.put("Heading (deg)", Math.toDegrees(robot.roadRunner.pose.heading.real) - 57.2958);
            packet.put("PoseX", robot.roadRunner.pose.position.x);
            packet.put("PoseY", robot.roadRunner.pose.position.y);
            packet.put("Deposit Height", Robot.HardwareDevices.depositSlide.getCurrentPosition());
            packet.put("Deposit Current (mA)", Robot.HardwareDevices.depositSlide.getCurrent(CurrentUnit.MILLIAMPS));
            packet.put("IntakeL Height", Robot.HardwareDevices.intakeSlideL.getCurrentPosition());
            packet.put("IntakeL Current (mA)", Robot.HardwareDevices.intakeSlideL.getCurrent(CurrentUnit.MILLIAMPS));
            packet.put("IntakeR Height", Robot.HardwareDevices.intakeSlideR.getCurrentPosition());
            packet.put("IntakeR Current (mA)", Robot.HardwareDevices.intakeSlideR.getCurrent(CurrentUnit.MILLIAMPS));
            packet.put("Deposit Magnet", Robot.HardwareDevices.depositMagnet.isPressed());
            dashboard.sendTelemetryPacket(packet);

            telemetry.addData("Heading", Math.toDegrees(robot.roadRunner.pose.heading.real) - 57.2958);
            telemetry.addData("PoseX", (robot.roadRunner.pose.position.x));
            telemetry.addData("PoseY", (robot.roadRunner.pose.position.y));
            telemetry.addLine();
            telemetry.addData("Deposit Height", Robot.HardwareDevices.depositSlide.getCurrentPosition());
            telemetry.addData("Deposit Current (mA)", Robot.HardwareDevices.depositSlide.getCurrent(CurrentUnit.MILLIAMPS));
            telemetry.addLine();
            telemetry.addData("IntakeL Height", (Robot.HardwareDevices.intakeSlideL.getCurrentPosition()));
            telemetry.addData("IntakeL Current (mA)", Robot.HardwareDevices.intakeSlideL.getCurrent(CurrentUnit.MILLIAMPS));
            telemetry.addLine();
            telemetry.addData("IntakeR Height", (Robot.HardwareDevices.intakeSlideR.getCurrentPosition()));
            telemetry.addData("IntakeR Current (mA)", Robot.HardwareDevices.intakeSlideR.getCurrent(CurrentUnit.MILLIAMPS));
            telemetry.addLine();
            telemetry.addData("Deposit Magnet", Robot.HardwareDevices.depositMagnet.isPressed());
            telemetry.update();
            //endregion
        }
    }
}
