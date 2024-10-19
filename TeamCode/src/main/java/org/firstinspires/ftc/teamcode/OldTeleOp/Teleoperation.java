//region Imports
package org.firstinspires.ftc.teamcode.OldTeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.main.Auto.RoadRunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.main.TeleOp.DepositThread;
import org.firstinspires.ftc.teamcode.main.TeleOp.IntakeThread;
//endregion

@TeleOp(name = "Drive")
@Config
public class Teleoperation extends LinearOpMode {

    //region Variables
    public static double intakePosUp = 0.26;
    public static double intakePosDown = 0.33;
    public static double intakePosDownBack = 0.35;

    public static double intakeYawMulti = 0.001;

    public static int intakeExtended = -1500;
    public static int intakeRetracted = 0;

    public static int depositRetracted = 0;
    public static int depositHighBasket = -1675;
    public static int depositLowBasket = -600;

    public static double intakePower = 1;
    public static double depositPower = 1;

    public static double intakeYawCenter = 0.6225;

    public static double intakeRollerSpeed = 1;

    public static double clawDownPos = 0.6075;
    public static double clawUpPos = 0.575;
    public static double clawBackPos = 0.42;
    public static double clawForwardPos = 0.5925;

    public static double clawAngleHorizontal = 0.27;
    public static double clawAngleVertical = 0.625;

    public static double clawClosed = 0.5;
    public static double clawOpen = 0;

    public static double intakePitchThreshold = 0.1;
    public static double intakeYawThreshold = 0.1;

    public static double yStickLMulti = 1;
    public static double xStickLMulti = 0.6;
    public static double xStickRMulti = 1;

//    public volatile float touchX = gamepad2.touchpad_finger_1_x;
//    public volatile float touchY = gamepad2.touchpad_finger_1_y;

    boolean driveToggle = false;
    //endregion

    @Override
    public void runOpMode() {

        //region Hardware Initialization & Encoding
        // All motors are goBlida Yellow Jacket 5203 with 435rpm
        // Roller is goBilda Speed, both claw are rev, and the rest are goBilda Torque servos

        DcMotorEx intakeSlide = hardwareMap.get(DcMotorEx.class, "intakeSlide");
        intakeSlide.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        intakeSlide.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            intakeSlide.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            intakeSlide.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        DcMotorEx depositSlide = hardwareMap.get(DcMotorEx.class, "depositSlide");
            depositSlide.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            depositSlide.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            depositSlide.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        DcMotorEx arm = hardwareMap.get(DcMotorEx.class, "arm");
            arm.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            arm.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            arm.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        DcMotorEx leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
            leftFront.setDirection(DcMotorEx.Direction.REVERSE);
        DcMotorEx rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        DcMotorEx leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
            leftBack.setDirection(DcMotorEx.Direction.REVERSE);
        DcMotorEx rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");

        Servo intakeYaw = hardwareMap.get(Servo.class, "intakeYaw");
        Servo intakePitch = hardwareMap.get(Servo.class, "intakePitch");
        CRServo intakeRoller = hardwareMap.get(CRServo.class, "intakeRoller");

        Servo clawArm = hardwareMap.get(Servo.class, "clawArm");
        Servo clawAngle = hardwareMap.get(Servo.class, "clawAngle");
        Servo claw = hardwareMap.get(Servo.class, "claw");
        //endregion

        //region Dashboard
        FtcDashboard dashboard = FtcDashboard.getInstance();
        //endregion

        //region IMU
        IMU imu = hardwareMap.get(IMU.class, "imu");
        YawPitchRollAngles robotOrientation;

        imu.initialize(
                new IMU.Parameters(
                        new RevHubOrientationOnRobot(
                                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                                RevHubOrientationOnRobot.UsbFacingDirection.UP
                        )
                )
        );
        imu.resetYaw();
        //endregion

        //region Pose2d
        Pose2d beginPose = new Pose2d(0, 0, 0); // TODO: Figure out what pos to start with (Changes depending on situation)
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);
        //endregion

        waitForStart();

        while (opModeIsActive()) {

            //region Pose2d Data
            // TODO: Fix
            double xPos = drive.pose.position.x;
            double yPos = drive.pose.position.y;
            double rotAngle = drive.pose.heading.real;

//            if (gamepad1.touchpad_finger_1) {
//                Actions.runBlocking(
//                        drive.actionBuilder(beginPose)
//                                .lineToXLinearHeading(touchY, 180)
//                                .build());
//            }
            //endregion

            //region Driving
            robotOrientation = imu.getRobotYawPitchRollAngles();
            double imuYaw = -robotOrientation.getYaw(AngleUnit.DEGREES);
            if (imuYaw < 0) {
                imuYaw += 360;
            }

            double xStickR = gamepad1.right_stick_x * xStickRMulti;
            double xStickL = gamepad1.left_stick_x;
            double yStickL = gamepad1.left_stick_y;

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
                leftFront.setPower(newYStickL + newXStickL + xStickR);
                leftBack.setPower(newYStickL - newXStickL + xStickR);
                rightFront.setPower(newYStickL - newXStickL - xStickR);
                rightBack.setPower(newYStickL + newXStickL - xStickR);
            } else {
                // Normal Drive
                leftFront.setPower(-yStickL + xStickL + xStickR);
                leftBack.setPower(-yStickL - xStickL + xStickR);
                rightFront.setPower(-yStickL - xStickL - xStickR);
                rightBack.setPower(-yStickL + xStickL - xStickR);
            }
            //endregion

            // TODO: Deposit slide controls
            // TODO: One button to drop in low or high net
            // TODO: One button pickup off wall
            // TODO: One button low chamber / high chamber
            // Based on field position change what the buttons do for these operations

            //region Intake Controls
            if (Math.abs(gamepad2.right_stick_x) > intakeYawThreshold) {
                intakeYaw.setPosition(intakeYaw.getPosition() - (gamepad2.right_stick_x * intakeYawMulti)); // TODO: Multiply to reduce how much the intake turns
            }

            // Intake samples
            if (gamepad2.right_bumper) {
                intakeRoller.setPower(intakeRollerSpeed);
            } else if (gamepad2.left_bumper) {
                intakeRoller.setPower(-intakeRollerSpeed);
            }

            if (gamepad2.cross) {
                intakePitch.setPosition(intakePosUp);
            } else if (gamepad2.circle) {
                intakePitch.setPosition(intakePosDown);
            }

            //endregion

            //region Claw Controls
            if (gamepad2.square) {
                claw.setPosition(clawOpen);
            } else if (gamepad2.triangle) {
                claw.setPosition(clawClosed);
            }
            //endregion

            //region Arm Controls
            if (gamepad1.right_trigger > 0) {
                arm.setPower(-gamepad1.right_trigger);
            } else if (gamepad1.left_trigger > 0) {
                arm.setPower(gamepad1.left_trigger);
            } else
                arm.setPower(0);
            //endregion

            //region Macros
            if (gamepad2.dpad_up) {
                intakePitch.setPosition(intakePosUp);
                intakeYaw.setPosition((intakeYawCenter) + 0.004);
                sleep(750);
                intakeSlide.setTargetPositionTolerance(5);
                intakeSlide.setTargetPosition(intakeExtended);
                intakeSlide.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                intakeSlide.setPower(intakePower);
                while (!(intakeSlide.getCurrentPosition() < -1400)) {
                    sleep(10);
                }
                intakeSlide.setPower(intakePower / 5);
                intakePitch.setPosition(intakePosDown);
                intakeYaw.setPosition((intakeYawCenter));
                intakeRoller.setPower(intakeRollerSpeed);
            }

            if (gamepad2.dpad_down) {
                intakePitch.setPosition(intakePosUp);
                intakeYaw.setPosition((intakeYawCenter) + 0.004);
                sleep(750);
                intakeSlide.setTargetPositionTolerance(5);
                intakeSlide.setTargetPosition(intakeRetracted);
                intakeSlide.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                intakeSlide.setPower(intakePower);
                while (!(intakeSlide.getCurrentPosition() > -100)) {
                    sleep(10);
                }
                intakePitch.setPosition(intakePosDown);
                intakeYaw.setPosition((intakeYawCenter));
                intakeRoller.setPower(0);
            }

            if (gamepad2.dpad_left) {
                claw.setPosition(clawOpen);
                clawArm.setPosition(clawDownPos);
                clawAngle.setPosition(clawAngleVertical);
                sleep(3000);
                claw.setPosition(clawClosed);
                sleep(1500);
                clawArm.setPosition(clawForwardPos);
                sleep(1000);
                depositSlide.setTargetPositionTolerance(5);
                depositSlide.setTargetPosition(depositHighBasket);
                depositSlide.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                depositSlide.setPower(depositPower);
                while (!(depositSlide.getCurrentPosition() < (depositHighBasket + 50))) {
                    sleep(10);
                }
                clawArm.setPosition(clawBackPos);
                sleep(2000);
                claw.setPosition(clawOpen);
            }
            if (gamepad1.circle && (clawArm.getPosition() == clawBackPos)) {
                claw.setPosition(clawOpen);
            }

            if (gamepad2.dpad_right) {
                clawArm.setPosition(clawDownPos);
                clawAngle.setPosition(clawAngleVertical);
                claw.setPosition(clawOpen);
                sleep(1000);
                depositSlide.setTargetPositionTolerance(5);
                depositSlide.setTargetPosition(depositRetracted);
                depositSlide.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                depositSlide.setPower(depositPower);
            }

            if ((!depositSlide.isBusy()) && (depositSlide.getTargetPosition() > -5)) {
                depositSlide.setPower(0);
            }

            if (gamepad2.ps) {
                intakeSlide.setPower(0);
                depositSlide.setPower(0);
            }

            if (gamepad1.ps) {
                intakeSlide.setPower(0);
                depositSlide.setPower(0);
            }
            //endregion

            //region Telemetry
            TelemetryPacket packet = new TelemetryPacket();
            packet.put("Touchpad X", gamepad2.touchpad_finger_1_x);
            packet.put("Touchpad Y", gamepad2.touchpad_finger_1_y);
            dashboard.sendTelemetryPacket(packet);

            telemetry.addData("X Position", xPos);
            telemetry.addData("Y Position", yPos);
            telemetry.addData("Rotation", rotAngle);
            telemetry.addLine();
            telemetry.addData("Claw", claw.getPosition());
            telemetry.addData("Claw Rotation", clawAngle.getPosition());
            telemetry.addLine();
            telemetry.addData("Deposit Height", depositSlide.getCurrentPosition());
            telemetry.addData("Intake Height", intakeSlide.getCurrentPosition());
            telemetry.addLine();
            telemetry.addData("Intake Yaw", intakeYaw.getPosition());
            telemetry.addData("Intake Velocity", intakeSlide.getVelocity());
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
            telemetry.addData("Touchpad X", gamepad1.touchpad_finger_1_x);
            telemetry.addData("Touchpad Y", gamepad1.touchpad_finger_1_y);
            telemetry.update();
            //endregion
        }
    }
}