package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class DepositSlide extends Robot.HardwareDevices {
    @Config
    public static class DepositSlidePosition {
        public static int retracted = 0;
        public static int highBasket = 2375;
        public static int lowBasket = 1000;
        public static int tolerance = 5;
        public static int stepRange = 50;
        public static int stopTolerance = 5;

    }
    @Config
    public static class DepositSlidePower {
        public static double stop = 0;
        public static double move = 1;
    }
    public void stop() {
        depositSlide.setPower(DepositSlidePower.stop);
    }
    public void move(int amount){
        depositSlide.setTargetPosition(amount);
        depositSlide.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        depositSlide.setPower(DepositSlidePower.move);
    }
    public void retract() {
        depositSlide.setTargetPosition(DepositSlidePosition.retracted);
        depositSlide.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        depositSlide.setPower(DepositSlidePower.move);
    }
    public void highBasket() {
        depositSlide.setTargetPosition(DepositSlidePosition.highBasket);
        depositSlide.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        depositSlide.setPower(DepositSlidePower.move);
    }
    public void lowBasket() {
        depositSlide.setTargetPosition(DepositSlidePosition.lowBasket);
        depositSlide.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        depositSlide.setPower(DepositSlidePower.move);
    }

    public void deposit() {
        try {
            depositSlide.setTargetPositionTolerance(DepositSlidePosition.tolerance);
            depositSlide.setTargetPosition(DepositSlidePosition.highBasket);
            depositSlide.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            depositSlide.setPower(DepositSlidePower.move);
            while (!(depositSlide.getCurrentPosition() > (DepositSlidePosition.highBasket - DepositSlidePosition.stepRange))) {
                Thread.sleep(10);
            }
            clawArm.setPosition(Claw.ClawPosition.back);
            Thread.sleep(750);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
    }
    public void condense() {
        try {
            clawArm.setPosition(Claw.ClawPosition.down);
            clawAngle.setPosition(Claw.ClawPosition.vertical);
            claw.setPosition(Claw.ClawPosition.open);
            Thread.sleep(1000);
            depositSlide.setTargetPositionTolerance(DepositSlidePosition.tolerance);
            depositSlide.setTargetPosition(DepositSlidePosition.retracted);
            depositSlide.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            depositSlide.setPower(DepositSlidePower.move);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
    }
}
