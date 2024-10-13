package org.firstinspires.ftc.teamcode.TeleOp;

import android.util.Log;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.Auto.RoadRunner.MecanumDrive;

@TeleOp(name = "Drive")
@Config
public class Drive extends LinearOpMode {

    public static double intakePosUp = 0.27;
    public static double intakePosDown = 0.35;

    public static double intakeYawMulti = 0.001;

    public static int intakeExtended = -1500;
    public static int intakeRetracted = -5;

    public static double intakePower = 1;

    public static double intakeYawCenter = 0.576;

    public static double intakeRollerSpeed = 1;

    public static double clawDownPos = 0.653;
    public static double clawUpPos = 0.79;

    public static double clawAngleVertical = 0.625;
    public static double clawAngleHorizontal = 0.5; // TODO: Find

    public static double clawClosed = 0.4;
    public static double clawOpen = 0;

    public static double intakePitchThreshold = 0.1;
    public static double intakeYawThreshold = 0.1;

    // TODO: Tune with driver
    public static double yStickLMulti = 0.6;
    public static double xStickLMulti = 0.75;
    public static double xStickRMulti = 0.85;

    boolean driveToggle = false;
    // TODO: Add telemetry into FTC dashboard

    @Override
    public void runOpMode() {

        // All motors are goBlida Yellow Jacket 5203 with 435rpm
        // Roller is goBilda Speed, both claw are rev, and the rest are goBilda Torque servos

        DcMotorEx intakeSlide = hardwareMap.get(DcMotorEx.class, "intakeSlide");
        intakeSlide.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        intakeSlide.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        intakeSlide.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        intakeSlide.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        DcMotorEx depositSlide = hardwareMap.get(DcMotorEx.class, "depositSlide");
        depositSlide.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        depositSlide.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        depositSlide.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        depositSlide.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

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

        IMU imu = hardwareMap.get(IMU.class, "imu");

        imu.initialize(
                new IMU.Parameters(
                        new RevHubOrientationOnRobot(
                                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                                RevHubOrientationOnRobot.UsbFacingDirection.UP
                        )
                )
        );

        YawPitchRollAngles robotOrientation;

        Pose2d beginPose = new Pose2d(0, 0, 0); // TODO: Figure out what pos to start with (Changes depending on situation)
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

        waitForStart();
        while (opModeIsActive()) {

            // TODO: Fix
            // IN TESTING
            double xPos = drive.pose.position.x;
            double yPos = drive.pose.position.y;
            // Figure out where the robot is to change
            double rotAngle = drive.pose.heading.real;

            robotOrientation = imu.getRobotYawPitchRollAngles();

            double imuYaw = robotOrientation.getYaw(AngleUnit.DEGREES);
            double imuPitch = robotOrientation.getPitch(AngleUnit.DEGREES);
            double imuRoll = robotOrientation.getRoll(AngleUnit.DEGREES);

            // Convert yaw from degrees to radians
            double yawRadians = Math.toRadians(imuYaw);

            double xStickR = xStickRMulti * gamepad1.right_stick_x;
            double xStickL = xStickLMulti * gamepad1.left_stick_x;
            double yStickL = -yStickLMulti * gamepad1.left_stick_y;

            // Rotate the joystick inputs based on IMU yaw
            double tempX = xStickL * Math.cos(yawRadians) - yStickL * Math.sin(yawRadians);
            double tempY = xStickL * Math.sin(yawRadians) + yStickL * Math.cos(yawRadians);
            double xStickRAdjusted = xStickR * Math.cos(yawRadians) - yStickL * Math.sin(yawRadians);

            // Trigger driveToggle
            if (gamepad1.dpad_left) {
                driveToggle = true;
            }
            if (gamepad1.dpad_right) {
                driveToggle = false;
            }

            if (driveToggle) {
                // Mecanum Drive
                leftFront.setPower(tempY + tempX + xStickRAdjusted);
                leftBack.setPower(tempY - tempX + xStickRAdjusted);
                rightFront.setPower(tempY - tempX - xStickRAdjusted);
                rightBack.setPower(tempY + tempX - xStickRAdjusted);
            } else {
                // Normal Drive
                leftFront.setPower(yStickL + xStickL + xStickR);
                leftBack.setPower(yStickL - xStickL + xStickR);
                rightFront.setPower(yStickL - xStickL - xStickR);
                rightBack.setPower(yStickL + xStickL - xStickR);
            }

            // Field Oriented Control
            // Fix???
            if (imuYaw < 0) {
                imuYaw += 360;
            }

            // Adjust motor powers based on robot orientation
            // Convert IMU yaw to radians
            double angleInRadians = Math.toRadians(imuYaw);

            double newForward = yStickL * Math.cos(angleInRadians) + xStickL * Math.sin(angleInRadians);
            double sidewaysVelocity = -yStickL * Math.sin(angleInRadians) + xStickL * Math.cos(angleInRadians);

//            if (gamepad2.right_trigger > 0) {
//                intakeSlide.setPower(-gamepad2.right_trigger);
//            } else if (gamepad2.left_trigger > 0) {
//                intakeSlide.setPower(gamepad2.left_trigger);
//            } else {
//                intakeSlide.setPower(0);
//            }

//            depositSlide.setPower(gamepad2.left_stick_y);

            // TODO: Deposit slide controls
            //
            // TODO: One button to drop in low or high net
            // TODO: One button pickup off wall
            // TODO: One button low chamber / high chamber
            // Based on field position change what the buttons do for these operations

            if (Math.abs(gamepad2.right_stick_x) > intakeYawThreshold) {
                intakeYaw.setPosition(intakeYaw.getPosition() - (gamepad2.right_stick_x * intakeYawMulti)); // TODO: Multiply to reduce how much the intake turns
            }

//            if (Math.abs(gamepad2.right_stick_y) > intakePitchThreshold) {
//                if (gamepad2.right_stick_y < -intakePitchThreshold) {
//                    intakePitch.setPosition(intakePosUp); // TODO: Flip after testing
//                } else if (gamepad2.right_stick_y > intakePitchThreshold) {
//                    intakePitch.setPosition(intakePosDown); // TODO: Flip after testing
//                }
//            }

            // Intake samples
            if (gamepad2.right_bumper) {
                intakeRoller.setPower(intakeRollerSpeed);
            } else if (gamepad2.left_bumper) {
                intakeRoller.setPower(-intakeRollerSpeed);
            }

            if (gamepad2.cross) {
                intakeYaw.setPosition(intakeYawCenter);
            } else if (gamepad2.circle) {
                clawAngle.setPosition(clawAngleHorizontal);
            }

            if (gamepad2.square) {
                claw.setPosition(clawOpen);
            } else if (gamepad2.triangle) {
                claw.setPosition(clawClosed);
            }

            if (gamepad2.dpad_up) {
                intakePitch.setPosition(intakePosUp);
                intakeYaw.setPosition((intakeYawCenter) + 0.003);
            }
            if (gamepad2.dpad_down) {
                intakePitch.setPosition(intakePosDown);
                intakeYaw.setPosition((intakeYawCenter));
            }

            if (gamepad2.share) {
                intakePitch.setPosition(intakePosUp);
                intakeYaw.setPosition((intakeYawCenter) + 0.003);
                sleep(750);
                intakeSlide.setTargetPosition(intakeExtended);
                intakeSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                intakeSlide.setPower(intakePower);
                while (!(intakeSlide.getCurrentPosition() < -1450)) {
                    sleep(10);
                }
                intakePitch.setPosition(intakePosDown);
                intakeYaw.setPosition((intakeYawCenter));
                intakeRoller.setPower(intakeRollerSpeed);
            }

            if (gamepad2.options) {
                intakePitch.setPosition(intakePosUp);
                intakeYaw.setPosition((intakeYawCenter) + 0.003);
                sleep(750);
                intakeSlide.setTargetPosition(intakeRetracted);
                intakeSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                intakeSlide.setPower(intakePower);
                while (!(intakeSlide.getCurrentPosition() > -100)) {
                    sleep(10);
                }
                intakePitch.setPosition(intakePosDown);
                intakeYaw.setPosition((intakeYawCenter));
                intakeRoller.setPower(0);
            }

            if (gamepad2.dpad_left) {
                clawArm.setPosition(clawUpPos);
            }
            if (gamepad2.dpad_right) {
                clawArm.setPosition(clawDownPos);
            }

            if (gamepad2.ps) {
                intakeSlide.setPower(0);
                depositSlide.setPower(0);
            }

            if (gamepad1.ps) {
                intakeSlide.setPower(0);
                depositSlide.setPower(0);
            }

            // FOR TESTING
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
            telemetry.addData("New F:", newForward);
            telemetry.addData("Side V: ", sidewaysVelocity);
            telemetry.addLine();
            telemetry.addData("IMU Pitch", imuPitch);
            telemetry.addData("IMU Yaw", imuYaw);
            telemetry.addData("IMU Roll", imuRoll);
            telemetry.update();
        }
    }
}
