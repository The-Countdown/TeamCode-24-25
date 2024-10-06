package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;

import org.firstinspires.ftc.teamcode.Auto.RoadRunner.MecanumDrive;

@TeleOp(name = "Drive")
public class Drive extends LinearOpMode {

    public static int superCoolVariable = 37;

    @Override
    public void runOpMode() {

        //All motors are goBlida Yellow Jacket 5203 with 435rpm
        //All servos except for claw and clawAngle are goBilda Dual Mode

        DcMotorEx intakeSlide = hardwareMap.get(DcMotorEx.class, "intakeSlide");
        DcMotorEx depositSlide = hardwareMap.get(DcMotorEx.class, "depositSlide");

        DcMotorEx leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        DcMotorEx rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        DcMotorEx leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        DcMotorEx rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");

        Servo intakeYaw = hardwareMap.get(Servo.class,"intakeYaw");
        Servo intakePitch = hardwareMap.get(Servo.class,"intakePitch");
        CRServo intakeRoller = hardwareMap.get(CRServo.class,"intakeRoller");

        Servo depositServo = hardwareMap.get(Servo.class,"depositServo");
        Servo clawAngle = hardwareMap.get(Servo.class,"clawAngle");
        Servo claw = hardwareMap.get(Servo.class,"claw");

        Pose2d beginPose = new Pose2d(0, 0, 0); //TODO: Figure out what pos to start with (Changes depending on situation)
        MecanumDrive drive = new MecanumDrive(hardwareMap,beginPose);

        waitForStart();

        while (opModeIsActive()) {

            double xPos = drive.pose.position.x;
            double yPos = drive.pose.position.y;
            double rotAngle = drive.pose.heading.real;

            double yStickL = -0.6 * gamepad1.right_stick_x;
            double xStickL = -0.75 * gamepad1.left_stick_x;
            double xStickR = 0.85 * gamepad1.left_stick_y;

            double intakeUpPos = 0; //TODO: Find this position
            double intakeDownPos = 0; //TODO: Find this position

            leftFront.setPower(yStickL + xStickL + xStickR); //TODO: Fix low-power motor
            leftBack.setPower(yStickL - xStickL + xStickR);
            rightFront.setPower(yStickL - xStickL - xStickR);
            rightBack.setPower(yStickL + xStickL - xStickR);
            intakeSlide.setPower(gamepad2.right_trigger);
            intakeSlide.setPower(-gamepad2.left_trigger);

            intakeYaw.setPosition(gamepad2.left_stick_x); //TODO: Multiply to reduce how much the intake turns

            if (gamepad2.right_stick_y > 0)
                intakePitch.setPosition(intakeUpPos);
            else if (gamepad2.right_stick_y < 0) {
                intakePitch.setPosition(intakeDownPos);
            }

            if (gamepad2.right_bumper)
                intakeRoller.setPower(1);
            else if (gamepad2.left_bumper) {
                intakeRoller.setPower(-1);
            }
            else
                intakeRoller.setPower(0);

            //TODO: Add limits so when the intake gets to a certain point it goes up and straightens

            telemetry.update();
            telemetry.addData("X Position",xPos);
            telemetry.addData("Y Position",yPos);
            telemetry.addData("Rotation",rotAngle);

        }
    }
}

