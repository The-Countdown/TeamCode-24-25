package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Robot {
    HardwareMap hardwareMap;
    Telemetry telemetry;
    LinearOpMode opMode;
    public static class HardwareDevices {
        public static DcMotorEx leftFront;
        public static DcMotorEx rightFront;
        public static DcMotorEx leftBack;
        public static DcMotorEx rightBack;

        public static DcMotorEx intakeSlide;
        public static DcMotorEx depositSlide;
        public static DcMotorEx arm;

        public static Servo intakePitch;
        public static Servo intakeYaw;
        public static CRServo intakeRoller;

        public static Servo claw;
        public static Servo clawArm;
        public static Servo clawAngle;

        public static Limelight3A limelight;
        public static IMU imu;
    }

    public Robot(HardwareMap hardwareMap, Telemetry telemetry, LinearOpMode opMode) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        this.opMode = opMode;

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
        HardwareDevices.intakeSlide = hardwareMap.get(DcMotorEx.class, "intakeSlide");
        HardwareDevices.depositSlide = hardwareMap.get(DcMotorEx.class, "depositSlide");
        HardwareDevices.arm = hardwareMap.get(DcMotorEx.class, "arm");

        // Servos
        HardwareDevices.intakeYaw = hardwareMap.get(Servo.class, "intakeYaw");
        HardwareDevices.intakePitch = hardwareMap.get(Servo.class, "intakePitch");
        HardwareDevices.intakeRoller = hardwareMap.get(CRServo.class, "intakeRoller");

        HardwareDevices.clawArm = hardwareMap.get(Servo.class, "clawArm");
        HardwareDevices.clawAngle = hardwareMap.get(Servo.class, "clawAngle");
        HardwareDevices.claw = hardwareMap.get(Servo.class, "claw");

        // Motor Directions
        HardwareDevices.leftFront.setDirection(DcMotorEx.Direction.REVERSE);
        HardwareDevices.leftBack.setDirection(DcMotorEx.Direction.REVERSE);
        HardwareDevices.intakeSlide.setDirection(DcMotorEx.Direction.REVERSE);
        HardwareDevices.depositSlide.setDirection(DcMotorEx.Direction.REVERSE);
        HardwareDevices.arm.setDirection(DcMotorEx.Direction.REVERSE);

        // Motor Modes and Settings
        HardwareDevices.leftFront.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        HardwareDevices.rightFront.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        HardwareDevices.leftBack.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        HardwareDevices.rightBack.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        HardwareDevices.intakeSlide.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        HardwareDevices.intakeSlide.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        HardwareDevices.intakeSlide.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        HardwareDevices.depositSlide.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        HardwareDevices.depositSlide.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        HardwareDevices.depositSlide.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        HardwareDevices.arm.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        HardwareDevices.arm.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        HardwareDevices.arm.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        HardwareDevices.limelight = hardwareMap.get(Limelight3A.class, "limelight");

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
        Pose2d initialPose = new Pose2d(0, 0, 0);
        this.drive = new Drive(hardwareMap, initialPose);
    }

    public Drive drive;
    public IntakeSlide intakeSlide = new IntakeSlide();
    public DepositSlide depositSlide = new DepositSlide();
    public Intake intake = new Intake();
    public Claw claw = new Claw();
    public Arm arm = new Arm();
}


