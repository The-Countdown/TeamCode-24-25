package org.firstinspires.ftc.teamcode.TeleOp;

import static org.firstinspires.ftc.teamcode.TeleOp.Drive.clawAngleVertical;
import static org.firstinspires.ftc.teamcode.TeleOp.Drive.clawBackPos;
import static org.firstinspires.ftc.teamcode.TeleOp.Drive.clawClosed;
import static org.firstinspires.ftc.teamcode.TeleOp.Drive.clawDownPos;
import static org.firstinspires.ftc.teamcode.TeleOp.Drive.clawForwardPos;
import static org.firstinspires.ftc.teamcode.TeleOp.Drive.clawOpen;
import static org.firstinspires.ftc.teamcode.TeleOp.Drive.depositHighBasket;
import static org.firstinspires.ftc.teamcode.TeleOp.Drive.depositPower;
import static org.firstinspires.ftc.teamcode.TeleOp.Drive.depositRetracted;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;


public class Deposit implements Runnable {
    private final String depSequences;
    private final HardwareMap hardwareMap;
    private final Gamepad gamepad1;
    private final Gamepad gamepad2;

    public Deposit(String depSequences, HardwareMap hardwareMap, Gamepad gamepad2, Gamepad gamepad1) {
        this.depSequences = depSequences;
        this.hardwareMap = hardwareMap;
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
    }

    @Override
    public void run() {

        DcMotorEx depositSlide = hardwareMap.get(DcMotorEx.class, "depositSlide");
        Servo clawArm = hardwareMap.get(Servo.class, "clawArm");
        Servo clawAngle = hardwareMap.get(Servo.class, "clawAngle");
        Servo claw = hardwareMap.get(Servo.class, "claw");

        while (true) {
            try {
                if (gamepad2.dpad_left) {
                    claw.setPosition(clawOpen);
                    clawArm.setPosition(clawDownPos);
                    clawAngle.setPosition(clawAngleVertical);
                    Thread.sleep(3000);
                    claw.setPosition(clawClosed);
                    Thread.sleep(1500);
                    clawArm.setPosition(clawForwardPos);
                    Thread.sleep(1000);
                    depositSlide.setTargetPositionTolerance(5);
                    depositSlide.setTargetPosition(depositHighBasket);
                    depositSlide.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                    depositSlide.setPower(depositPower);
                    while (!(depositSlide.getCurrentPosition() < (depositHighBasket + 50))) {
                        Thread.sleep(10);
                    }
                    clawArm.setPosition(clawBackPos);
                    Thread.sleep(2000);
                    claw.setPosition(clawOpen);
                }
                if (gamepad1.b && (clawArm.getPosition() == clawBackPos)) {
                    claw.setPosition(clawOpen);
                }

                if (gamepad2.dpad_right) {
                    clawArm.setPosition(clawDownPos);
                    clawAngle.setPosition(clawAngleVertical);
                    claw.setPosition(clawOpen);
                    Thread.sleep(1000);
                    depositSlide.setTargetPositionTolerance(5);
                    depositSlide.setTargetPosition(depositRetracted);
                    depositSlide.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                    depositSlide.setPower(depositPower);
                }

                if ((!depositSlide.isBusy()) && (depositSlide.getTargetPosition() > -5)) {
                    depositSlide.setPower(0);
                }
            } catch (Exception e) {
                throw new RuntimeException(e);
            }
        }
    }
}
