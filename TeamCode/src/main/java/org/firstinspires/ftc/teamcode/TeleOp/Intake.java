package org.firstinspires.ftc.teamcode.TeleOp;

import static org.firstinspires.ftc.teamcode.TeleOp.Drive.intakeExtended;
import static org.firstinspires.ftc.teamcode.TeleOp.Drive.intakePosDown;
import static org.firstinspires.ftc.teamcode.TeleOp.Drive.intakePosUp;
import static org.firstinspires.ftc.teamcode.TeleOp.Drive.intakePower;
import static org.firstinspires.ftc.teamcode.TeleOp.Drive.intakeRetracted;
import static org.firstinspires.ftc.teamcode.TeleOp.Drive.intakeRollerSpeed;
import static org.firstinspires.ftc.teamcode.TeleOp.Drive.intakeYawCenter;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;


class Intake implements Runnable {
    private final String inSequences;
    private final HardwareMap hardwareMap;
    private final Gamepad gamepad2;

    public Intake(String inSequences, HardwareMap hardwareMap, Gamepad gamepad2) {
        this.inSequences = inSequences;
        this.hardwareMap = hardwareMap;
        this.gamepad2 = gamepad2;
    }

    @Override
    public void run() {

        DcMotorEx intakeSlide = hardwareMap.get(DcMotorEx.class, "intakeSlide");
            intakeSlide.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            intakeSlide.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            intakeSlide.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            intakeSlide.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        Servo intakeYaw = hardwareMap.get(Servo.class, "intakeYaw");
        Servo intakePitch = hardwareMap.get(Servo.class, "intakePitch");
        CRServo intakeRoller = hardwareMap.get(CRServo.class, "intakeRoller");

        while (true) {
            try {
                if (gamepad2.dpad_up) {
                    intakePitch.setPosition(intakePosUp);
                    intakeYaw.setPosition((intakeYawCenter) + 0.004);
                    Thread.sleep(750);
                    intakeSlide.setTargetPositionTolerance(5);
                    intakeSlide.setTargetPosition(intakeExtended);
                    intakeSlide.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                    intakeSlide.setPower(intakePower);
                    while (!(intakeSlide.getCurrentPosition() < -1400)) {
                        Thread.sleep(10);
                    }
                    intakeSlide.setPower(intakePower / 5);
                    intakePitch.setPosition(intakePosDown);
                    intakeYaw.setPosition((intakeYawCenter));
                    intakeRoller.setPower(intakeRollerSpeed);
                }

                if (gamepad2.dpad_down) {
                    intakePitch.setPosition(intakePosUp);
                    intakeYaw.setPosition((intakeYawCenter) + 0.003);
                    Thread.sleep(750);
                    intakeSlide.setTargetPositionTolerance(5);
                    intakeSlide.setTargetPosition(intakeRetracted);
                    intakeSlide.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                    intakeSlide.setPower(intakePower);
                    while (!(intakeSlide.getCurrentPosition() > -100)) {
                        Thread.sleep(10);
                    }
                    intakePitch.setPosition(intakePosDown);
                    intakeYaw.setPosition((intakeYawCenter));
                    intakeRoller.setPower(0);
                }
            } catch (Exception e) {
                e.printStackTrace();
            }
        }
    }
}