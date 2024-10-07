package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.Auto.RoadRunner.MecanumDrive;

@TeleOp(name = "Drive")
@Config
public class Drive extends LinearOpMode {

    public static double intakePosUp = 0.42;
    public static double intakePosDown = 0.48;

    public static double intakeYawMulti = 0.1;

    public static double intakeRollerSpeed = 1;

    public static double clawDownPos = 0.475;
    public static double clawUpPos = 0.6; //TODO: Tune to field

    public static double clawAngleVertical = 0.5; //TODO: Find
    public static double clawAngleHorizontal = 0.5; //TODO: Find

    public static double clawClosed = 0.5; //TODO: Find
    public static double clawOpen = 0.5; //TODO: Find

    public static double intakePitchThreshold = 0.1;
    public static double intakeYawThreshold = 0.1;

    //TODO: Tune with driver
    public static double yStickLMulti = -0.6;
    public static double xStickLMulti = -0.75;
    public static double xStickRMulti = 0.85;

    @Override
    public void runOpMode() {

        //All motors are goBlida Yellow Jacket 5203 with 435rpm
        //All servos except for claw and clawAngle are goBilda Dual Mode
        //Roller is goBilda Speed, both claw are rev, and the rest are goBilda Torque

        DcMotorEx intakeSlide = hardwareMap.get(DcMotorEx.class, "intakeSlide");
        DcMotorEx depositSlide = hardwareMap.get(DcMotorEx.class, "depositSlide");

        DcMotorEx leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        DcMotorEx rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        DcMotorEx leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        DcMotorEx rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");

        Servo intakeYaw = hardwareMap.get(Servo.class,"intakeYaw"); //TODO: Reprogram to default (Need different programmer?)
        Servo intakePitch = hardwareMap.get(Servo.class,"intakePitch");
        CRServo intakeRoller = hardwareMap.get(CRServo.class,"intakeRoller");

        Servo depositServo = hardwareMap.get(Servo.class,"depositServo");
        Servo clawAngle = hardwareMap.get(Servo.class,"clawAngle"); //TODO: Reprogram to default
        Servo claw = hardwareMap.get(Servo.class,"claw"); //TODO: Reprogram to default

        Pose2d beginPose = new Pose2d(0, 0, 0); //TODO: Figure out what pos to start with (Changes depending on situation)
        MecanumDrive drive = new MecanumDrive(hardwareMap,beginPose);

        double intakeYawStartPos = intakeYaw.getPosition();
        double intakePitchStartPos = intakePitch.getPosition(); //TODO: Incorporate

        waitForStart();

        while (opModeIsActive()) {

            //TODO: Fix
            //IN TESTING
            double xPos = drive.pose.position.x;
            double yPos = drive.pose.position.y;
            double rotAngle = drive.pose.heading.real;

            double xStickR = xStickRMulti * gamepad1.right_stick_x;
            double xStickL = xStickLMulti * gamepad1.left_stick_x;
            double yStickL = yStickLMulti * gamepad1.left_stick_y;

            leftFront.setPower(xStickR + xStickL + yStickL); //TODO: Fix low-power motor
            leftBack.setPower(xStickR - xStickL + yStickL);
            rightFront.setPower(xStickR - xStickL - yStickL);
            rightBack.setPower(xStickR + xStickL - yStickL);

            intakeSlide.setPower(gamepad2.right_trigger);
            intakeSlide.setPower(-gamepad2.left_trigger);

            depositSlide.setPower(gamepad2.left_stick_y);

            depositServo.setPosition(clawUpPos); //TODO: Bind to drop-off sequence

            if (Math.abs(gamepad2.right_stick_x) > intakeYawThreshold)
                intakeYaw.setPosition(intakeYawStartPos + (gamepad2.right_stick_x * intakeYawMulti)); //TODO: Multiply to reduce how much the intake turns

            if (Math.abs(gamepad2.right_stick_y) > intakePitchThreshold) {
                if (gamepad2.right_stick_y < -intakePitchThreshold)
                    intakePitch.setPosition(intakePosUp);
                else if (gamepad2.right_stick_y > intakePitchThreshold)
                    intakePitch.setPosition(intakePosDown);
            }

            if (gamepad2.right_bumper)
                intakeRoller.setPower(intakeRollerSpeed);
            else if (gamepad2.left_bumper) {
                intakeRoller.setPower(-intakeRollerSpeed);
            }
            else
                intakeRoller.setPower(0);

            if (gamepad2.cross)
                clawAngle.setPosition(clawAngleVertical);
            else if (gamepad2.circle)
                clawAngle.setPosition(clawAngleHorizontal);

            if (gamepad2.square)
                claw.setPosition(clawOpen);
            else if (gamepad2.triangle)
                claw.setPosition(clawClosed);


            //TODO: Add limits so when the intake gets to a certain point it goes up and straightens

            //IN TESTING
            telemetry.addData("X Position",xPos);
            telemetry.addData("Y Position",yPos);
            telemetry.addData("Rotation",rotAngle);

            telemetry.addData("Claw",claw.getPosition());
            telemetry.addData("Claw Rotation",clawAngle.getPosition());
            telemetry.addData("Deposit Height",depositSlide.getCurrentPosition());
            telemetry.addData("X",gamepad2.cross);
            telemetry.update();
        }
    }
}

