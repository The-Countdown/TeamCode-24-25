package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.CheckResult;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.main.Auto.RoadRunner.MecanumDrive;

import java.util.Date;

public class Robot {
    public static Robot rb;
    public static boolean hasResetEncoders = false;
    public static Pose2d teleOpStart = new Pose2d(0,0,0);
    public static Date teleOpStartDate = new Date(new Date().getTime() - 31556952000L);
    public static LimeLight.Pipelines color = LimeLight.Pipelines.Red;
    public boolean driveAvailable = true;
    public boolean isAuto = false;
    public Pose2d beginPose;
    public MecanumDrive roadRunner;
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

        public static ServoImplEx intakePitchL;
        public static ServoImplEx intakePitchR;
        public static ServoImplEx intakeClawAngle;
        public static ServoImplEx intakeCoaxialPitch;
        public static ServoImplEx intakeClaw;

        public static ServoImplEx depositClaw;
        public static ServoImplEx depositClawArmTop;
        public static ServoImplEx depositClawArmBottom;
        public static ServoImplEx depositClawAngle;

        public static TouchSensor depositMagnet;
        public static TouchSensor intakeMagnetL;
        public static TouchSensor intakeMagnetR;
        public static Limelight3A limelight;
        public static IMU imu;
        public static RevColorSensorV3 flashLight;
//        public static VoltageSensor voltageSensor;
    }

    public Robot(LinearOpMode opMode, Pose2d beginPose) {
        rb = this;

        this.opMode = opMode;
        this.hardwareMap = opMode.hardwareMap;
        this.telemetry = opMode.telemetry;
        this.beginPose = beginPose;

        HardwareDevices.depositMagnet = hardwareMap.get(TouchSensor.class, "depositMagnet");
        HardwareDevices.intakeMagnetL = hardwareMap.get(TouchSensor.class, "intakeMagnetL");
        HardwareDevices.intakeMagnetR = hardwareMap.get(TouchSensor.class, "intakeMagnetR");
        HardwareDevices.flashLight = hardwareMap.get(RevColorSensorV3.class, "flashLight");
        //HardwareDevices.voltageSensor = hardwareMap.get(VoltageSensor.class, "voltageSensor");
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

        // Servos
        HardwareDevices.intakeClaw = hardwareMap.get(ServoImplEx.class, "intakeClaw");
        HardwareDevices.intakePitchL = hardwareMap.get(ServoImplEx.class, "intakePitchL");
        HardwareDevices.intakePitchR = hardwareMap.get(ServoImplEx.class, "intakePitchR");
        HardwareDevices.intakeClawAngle = hardwareMap.get(ServoImplEx.class, "intakeClawAngle");
        HardwareDevices.intakeCoaxialPitch = hardwareMap.get(ServoImplEx.class, "intakeCoaxialPitch");

        HardwareDevices.depositClawArmTop = hardwareMap.get(ServoImplEx.class, "depositClawArmTop");
        HardwareDevices.depositClawArmBottom = hardwareMap.get(ServoImplEx.class, "depositClawArmBottom");
        HardwareDevices.depositClawAngle = hardwareMap.get(ServoImplEx.class, "depositClawAngle");
        HardwareDevices.depositClaw = hardwareMap.get(ServoImplEx.class, "depositClaw");

        // Motor Directions
        HardwareDevices.leftFront.setDirection(DcMotorEx.Direction.REVERSE);
        HardwareDevices.leftBack.setDirection(DcMotorEx.Direction.REVERSE);
        HardwareDevices.intakeSlideL.setDirection(DcMotorEx.Direction.REVERSE);
        HardwareDevices.depositSlide.setDirection(DcMotorEx.Direction.REVERSE);

        // Motor Modes and Settings
        HardwareDevices.leftFront.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        HardwareDevices.rightFront.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        HardwareDevices.leftBack.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        HardwareDevices.rightBack.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        HardwareDevices.intakeSlideL.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        HardwareDevices.intakeSlideR.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        HardwareDevices.depositSlide.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        if (beginPose != null) {
            isAuto = true;
        }

        if (beginPose != null || !hasResetEncoders) {
            HardwareDevices.intakeSlideL.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            HardwareDevices.intakeSlideL.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

            HardwareDevices.intakeSlideR.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            HardwareDevices.intakeSlideR.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

            HardwareDevices.depositSlide.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            HardwareDevices.depositSlide.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

            hasResetEncoders = true;
        }

        HardwareDevices.intakeSlideL.setTargetPosition(HardwareDevices.intakeSlideL.getCurrentPosition());
        HardwareDevices.intakeSlideR.setTargetPosition(HardwareDevices.intakeSlideR.getCurrentPosition());
        HardwareDevices.depositSlide.setTargetPosition(HardwareDevices.depositSlide.getCurrentPosition());

        HardwareDevices.imu = hardwareMap.get(IMU.class, "imu");
        HardwareDevices.imu.initialize(
                new IMU.Parameters(
                        new RevHubOrientationOnRobot(
                                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                                RevHubOrientationOnRobot.UsbFacingDirection.UP
                        )
                )
        );

        if (beginPose == null && (new Date().getTime() - teleOpStartDate.getTime()) < 30000) {
            beginPose = teleOpStart;
        } else {
            HardwareDevices.imu.resetYaw();
        }

        if (beginPose == null) {
            beginPose = new Pose2d(0, 0, Math.toRadians(0));
        }

        roadRunner = new MecanumDrive(hardwareMap, beginPose);
        roadRunner.updatePoseEstimate();
    }

    public Robot(LinearOpMode opMode) {
        this(opMode, null);
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

    public Drive drive = new Drive(this);
    public IntakeSlide intakeSlide = new IntakeSlide(this);
    public DepositSlide depositSlide = new DepositSlide(this);
    public Intake intake = new Intake(this);
    public Outtake outtake = new Outtake(this);
    public LimeLight limeLight = new LimeLight(this);
}


