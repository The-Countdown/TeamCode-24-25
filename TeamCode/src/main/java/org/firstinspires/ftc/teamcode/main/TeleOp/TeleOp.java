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

import java.text.SimpleDateFormat;
import java.util.Calendar;
import java.util.Locale;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOp")
@Config
public class TeleOp extends LinearOpMode {
    public static double yStickLMulti = 0.4;
    public static double xStickLMulti = 0.75;
    public static double xStickRMulti = 0.3;
    public static boolean depositMagnetPressed = false;
    public static boolean intakeMagnetLPressed = false;
    public static boolean intakeMagnetRPressed = false;
    public static String currentDateTime;

    @Override
    public void runOpMode() {
        Robot robot = new Robot(this);

        SimpleDateFormat sdf = new SimpleDateFormat("yyyy-MM-dd HH:mm:ss", Locale.getDefault());

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

        robot.limeLight.limeLightInit(0,100);
        LimeLightThread limeLightThread = new LimeLightThread(this, robot);
        Thread limeLight = new Thread(limeLightThread);
        limeLight.start();

        while (opModeIsActive()) {
            if (gamepad1.options || gamepad2.options) robot.driveAvailable = true;

            Calendar calendar = Calendar.getInstance();
            calendar.add(Calendar.MINUTE, -2);
            calendar.add(Calendar.SECOND, -15);
            currentDateTime = sdf.format(calendar.getTime());

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

/*            if (Robot.HardwareDevices.intakeMagnetL.isPressed()) {
                if (!intakeMagnetLPressed) {
                    Robot.HardwareDevices.intakeSlideL.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
                    Robot.HardwareDevices.intakeSlideL.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
                    Robot.HardwareDevices.intakeSlideL.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                    intakeMagnetLPressed = true;
                }
            } else {
                intakeMagnetLPressed = false;
            }

            if (Robot.HardwareDevices.intakeMagnetR.isPressed()) {
                if (!intakeMagnetRPressed) {
                    Robot.HardwareDevices.intakeSlideR.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
                    Robot.HardwareDevices.intakeSlideR.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
                    Robot.HardwareDevices.intakeSlideR.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                    intakeMagnetRPressed = true;
                }
            } else {
                intakeMagnetRPressed = false;
            }*/


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
            packet.put("Heading (deg)", Math.toDegrees(robot.roadRunner.pose.heading.toDouble()));
            packet.put("PoseX", robot.roadRunner.pose.position.x);
            packet.put("PoseY", robot.roadRunner.pose.position.y);
//            packet.put("Voltage", Robot.HardwareDevices.voltageSensor.getVoltage());
            packet.put("Deposit Height", Robot.HardwareDevices.depositSlide.getCurrentPosition());
            packet.put("Deposit Current (mA)", Robot.HardwareDevices.depositSlide.getCurrent(CurrentUnit.MILLIAMPS));
            packet.put("IntakeL Height", Robot.HardwareDevices.intakeSlideL.getCurrentPosition());
            packet.put("IntakeL Current (mA)", Robot.HardwareDevices.intakeSlideL.getCurrent(CurrentUnit.MILLIAMPS));
            packet.put("IntakeR Height", Robot.HardwareDevices.intakeSlideR.getCurrentPosition());
            packet.put("IntakeR Current (mA)", Robot.HardwareDevices.intakeSlideR.getCurrent(CurrentUnit.MILLIAMPS));
            packet.put("Deposit Magnet", Robot.HardwareDevices.depositMagnet.isPressed());
            dashboard.sendTelemetryPacket(packet);

            telemetry.addData("Date and Time", currentDateTime);
            telemetry.addData("Heading", Math.toDegrees(robot.roadRunner.pose.heading.toDouble()));
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
            telemetry.addLine();
            telemetry.addData("Magnitude", DriveThread.magnitudeL);

            double height = 6.3; // height of the camera in inches
            double moveAmountY = robot.limeLight.getLimeLightResult().getTy();
            double yDistance = height * Math.tan(Math.toRadians(moveAmountY));
            telemetry.addData("move amount", (yDistance*90));
            telemetry.addData("move target", (int)(Robot.HardwareDevices.intakeSlideL.getCurrentPosition() + (yDistance*90)));

            //telemetry.update();
            //endregion
        }
        Robot.hasResetEncoders = false;
    }
}
