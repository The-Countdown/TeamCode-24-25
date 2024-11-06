package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.CheckResult;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.main.Auto.RoadRunner.MecanumDrive;

public class Robot {
    public static Robot rb;
    public static Pose2d currentPose;
    public Pose2d beginPose;
    public MecanumDrive dreadDrive;
    HardwareMap hardwareMap;
    Telemetry telemetry;
    LinearOpMode opMode;
    public static class HardwareDevices {
        public static DcMotorEx leftFront;
        public static DcMotorEx rightFront;
        public static DcMotorEx leftBack;
        public static DcMotorEx rightBack;

        public static DcMotorEx intakeSlideL;
        public static DcMotorEx intakeSlideR;
        public static DcMotorEx depositSlide;
        public static DcMotorEx arm;

        public static Servo intakePitchL;
        public static Servo intakePitchR;
        public static Servo intakeYaw;
        public static CRServo intakeRoller;

        public static Servo claw;
        public static Servo clawArm;
        public static Servo clawAngle;

        public static TouchSensor depositMagnet;
        public static Limelight3A limelight;
        public static IMU imu;
    }

    public Robot(LinearOpMode opMode) {
        rb = this;

        this.opMode = opMode;
        this.hardwareMap = opMode.hardwareMap;
        this.telemetry = opMode.telemetry;

        HardwareDevices.depositMagnet = hardwareMap.get(TouchSensor.class, "depositMagnet");

        HardwareDevices.limelight = hardwareMap.get(Limelight3A.class, "limelight");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Drive base motors
        HardwareDevices.leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        HardwareDevices.rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        HardwareDevices.leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        HardwareDevices.rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");

        // Mechanism motors
        HardwareDevices.intakeSlideL = hardwareMap.get(DcMotorEx.class, "intakeSlideL");
        HardwareDevices.intakeSlideR = hardwareMap.get(DcMotorEx.class, "intakeSlideR");
        HardwareDevices.depositSlide = hardwareMap.get(DcMotorEx.class, "depositSlide");
        HardwareDevices.arm = hardwareMap.get(DcMotorEx.class, "arm");

        // Servos
        HardwareDevices.intakeYaw = hardwareMap.get(Servo.class, "intakeYaw");
        HardwareDevices.intakePitchL = hardwareMap.get(Servo.class, "intakePitchL");
        HardwareDevices.intakePitchR = hardwareMap.get(Servo.class, "intakePitchR");
        HardwareDevices.intakeRoller = hardwareMap.get(CRServo.class, "intakeRoller");

        HardwareDevices.clawArm = hardwareMap.get(Servo.class, "clawArm");
        HardwareDevices.clawAngle = hardwareMap.get(Servo.class, "clawAngle");
        HardwareDevices.claw = hardwareMap.get(Servo.class, "claw");

        // Motor Directions
        HardwareDevices.leftFront.setDirection(DcMotorEx.Direction.REVERSE);
        HardwareDevices.leftBack.setDirection(DcMotorEx.Direction.REVERSE);
        HardwareDevices.intakeSlideL.setDirection(DcMotorEx.Direction.REVERSE);
        HardwareDevices.depositSlide.setDirection(DcMotorEx.Direction.REVERSE);
        HardwareDevices.arm.setDirection(DcMotorEx.Direction.REVERSE);

        // Motor Modes and Settings
        HardwareDevices.leftFront.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        HardwareDevices.rightFront.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        HardwareDevices.leftBack.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        HardwareDevices.rightBack.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        HardwareDevices.intakeSlideL.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        HardwareDevices.intakeSlideL.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        HardwareDevices.intakeSlideL.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        HardwareDevices.intakeSlideR.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        HardwareDevices.intakeSlideR.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        HardwareDevices.intakeSlideR.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        HardwareDevices.depositSlide.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        HardwareDevices.depositSlide.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        HardwareDevices.depositSlide.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        HardwareDevices.arm.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        HardwareDevices.arm.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        HardwareDevices.arm.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        HardwareDevices.imu = hardwareMap.get(IMU.class, "imu");
        HardwareDevices.imu.initialize(
                new IMU.Parameters(
                        new RevHubOrientationOnRobot(
                                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                                RevHubOrientationOnRobot.UsbFacingDirection.UP
                        )
                )
        );
        HardwareDevices.imu.resetYaw();

        beginPose = new Pose2d(0, 0, Math.toRadians(0));
        dreadDrive = new MecanumDrive(hardwareMap, beginPose);
    }
    @CheckResult
    public boolean safeSleep(double milliseconds) {
        double startTime = System.currentTimeMillis();
        while (System.currentTimeMillis() - startTime < milliseconds) {
            opMode.idle();
            if (opMode.isStopRequested()) {
                return false;
            }
        }

        return true;
    }

    public void updatePose() {
        currentPose = dreadDrive.pose;
    }

    public Drive drive;
    public IntakeSlide intakeSlide = new IntakeSlide();
    public DepositSlide depositSlide = new DepositSlide();
    public IntakeClaw intakeClaw = new IntakeClaw();
    public DepositClaw depositClaw = new DepositClaw();
    public Arm arm = new Arm();
}


