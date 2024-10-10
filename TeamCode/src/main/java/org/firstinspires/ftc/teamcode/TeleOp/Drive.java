package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
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

    public static double intakePosUp = 0.58;
    public static double intakePosDown = 0.5;

    public static double intakeYawMulti = 0.001;

    public static int intakeExtended = -1000;

    public static int intakeVelocity = -300;

    public static double intakeYawCenter = 0.55;

    public static double intakeRollerSpeed = 1;

    public static double clawDownPos = 0.475;
    public static double clawUpPos = 0.6; // TODO: Tune to field

    public static double clawAngleVertical = 0.5; // TODO: Find
    public static double clawAngleHorizontal = 0.5; // TODO: Find

    public static double clawClosed = 0.5; // TODO: Find
    public static double clawOpen = 0.5; // TODO: Find

    public static double intakePitchThreshold = 0.1;
    public static double intakeYawThreshold = 0.1;

    // TODO: Tune with driver
    public static double yStickLMulti = 0.6;
    public static double xStickLMulti = 0.75;
    public static double xStickRMulti = 0.85;

    // TODO: Add telemetry into FTC dashboard

    @Override
    public void runOpMode() {

        // All motors are goBlida Yellow Jacket 5203 with 435rpm
        // Roller is goBilda Speed, both claw are rev, and the rest are goBilda Torque servos

        DcMotorEx intakeSlide = hardwareMap.get(DcMotorEx.class, "intakeSlide");
            intakeSlide.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            intakeSlide.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        DcMotorEx depositSlide = hardwareMap.get(DcMotorEx.class, "depositSlide");

        DcMotorEx leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        DcMotorEx rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        DcMotorEx leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        DcMotorEx rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");

        Servo intakeYaw = hardwareMap.get(Servo.class,"intakeYaw");
        Servo intakePitch = hardwareMap.get(Servo.class,"intakePitch");
        CRServo intakeRoller = hardwareMap.get(CRServo.class,"intakeRoller");

        Servo clawArm = hardwareMap.get(Servo.class,"clawArm");
        Servo clawAngle = hardwareMap.get(Servo.class,"clawAngle");
        Servo claw = hardwareMap.get(Servo.class,"claw");

        IMU imu = hardwareMap.get(IMU.class,"imu");

        imu.initialize(
                new IMU.Parameters(
                        new RevHubOrientationOnRobot(
                                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                        )
                )
        );

        YawPitchRollAngles robotOrientation;
        robotOrientation = imu.getRobotYawPitchRollAngles();

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

            double imuYaw   = robotOrientation.getYaw(AngleUnit.DEGREES);
            double imuPitch = robotOrientation.getPitch(AngleUnit.DEGREES);
            double imuRoll  = robotOrientation.getRoll(AngleUnit.DEGREES);

            double xStickR = xStickRMulti * gamepad1.right_stick_x;
            double xStickL = -xStickLMulti * gamepad1.left_stick_x;
            double yStickL = -yStickLMulti * gamepad1.left_stick_y; // TODO: Make negative after testing

            // Mecanum Drive
//            leftFront.setPower(xStickR + xStickL + yStickL);
//            leftBack.setPower(xStickR - xStickL + yStickL);
//            rightFront.setPower(xStickR - xStickL - yStickL);
//            rightBack.setPower(xStickR + xStickL - yStickL);

            // Field Oriented Control
            // Fix???
            if (imuYaw < 0) {
                imuYaw += 360;
            }

            // Adjust motor powers based on robot orientation
            double angleInRadians  = Math.toRadians(imuYaw);
            double newForward = yStickL * Math.cos(angleInRadians) +
                    xStickL * Math.sin(angleInRadians);
            double sidewaysVelocity = -yStickL * Math.sin(angleInRadians) +
                    xStickL * Math.cos(angleInRadians);

            // TODO: Check if it works
            leftFront.setPower(-(newForward + sidewaysVelocity));
            rightFront.setPower(newForward - sidewaysVelocity);
            leftBack.setPower(-(newForward - sidewaysVelocity));
            rightBack.setPower(newForward + sidewaysVelocity);

            if (gamepad2.right_trigger > 0); {
                intakeSlide.setPower(-gamepad2.right_trigger);
            }
            if (gamepad2.left_trigger > 0) {
                intakeSlide.setPower(gamepad2.left_trigger);
            }

            depositSlide.setPower(gamepad2.left_stick_y);
            // TODO: Deposit slide controls
            //
            // TODO: One button to drop in low or high net
            // TODO: One button pickup off wall
            // TODO: One button low chamber / high chamber
            // Based on field position change what the buttons do for these operations

            if (Math.abs(gamepad2.right_stick_x) > intakeYawThreshold) {
                intakeYaw.setPosition(intakeYaw.getPosition() - (gamepad2.right_stick_x * intakeYawMulti)); // TODO: Multiply to reduce how much the intake turns
            }

            if (Math.abs(gamepad2.right_stick_y) > intakePitchThreshold) {
                if (gamepad2.right_stick_y < -intakePitchThreshold) {
                    intakePitch.setPosition(intakePosUp); // TODO: Flip after testing
                } else if (gamepad2.right_stick_y > intakePitchThreshold) {
                    intakePitch.setPosition(intakePosDown); // TODO: Flip after testing
                }
            }

            // Intake samples
            if (gamepad2.right_bumper) {
                intakeRoller.setPower(intakeRollerSpeed);
            } else if (gamepad2.left_bumper) {
                intakeRoller.setPower(-intakeRollerSpeed);
            }
            else
                intakeRoller.setPower(0);

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
                intakePitch.setPosition(intakePosDown);
                intakeYaw.setPosition(intakeYawCenter);
                intakeSlide.setTargetPosition(intakeExtended);
                intakeSlide.setVelocity(intakeVelocity);
            }

            // FOR TESTING
            telemetry.addData("X Position", xPos);
            telemetry.addData("Y Position", yPos);
            telemetry.addData("Rotation", rotAngle);

            telemetry.addData("Claw", claw.getPosition());
            telemetry.addData("Claw Rotation", clawAngle.getPosition());
            telemetry.addData("Deposit Height", depositSlide.getCurrentPosition());
            telemetry.addData("Intake Height", intakeSlide.getCurrentPosition());
            telemetry.addData("Intake Yaw", intakeYaw.getPosition());
            telemetry.addData("Intake Velocity", intakeSlide.getVelocity());
            telemetry.addData("New F:", newForward);
            telemetry.addData("Side V: ", sidewaysVelocity);
            telemetry.update();
        }
    }
}

