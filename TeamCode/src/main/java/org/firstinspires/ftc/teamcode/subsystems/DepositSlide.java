package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class DepositSlide extends Robot.HardwareDevices {
    @Config
    public static class DepositSlidePosition {
        public static int retracted = 20;
        public static int highBasket = 2500;
        public static int lowBasket = 1500; //TODO: Check?
        public static int specimenWall = 700;
        public static int specimenBar = 1250;
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
        depositSlide.setTargetPositionTolerance(DepositSlidePosition.tolerance);
        depositSlide.setTargetPosition(amount);
        depositSlide.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        depositSlide.setPower(DepositSlidePower.move);
    }
    public void retract() {
        depositSlide.setTargetPositionTolerance(DepositSlidePosition.tolerance);
        depositSlide.setTargetPosition(DepositSlidePosition.retracted);
        depositSlide.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        depositSlide.setPower(DepositSlidePower.move);
    }
    public void specimenWall() {
        depositSlide.setTargetPositionTolerance(DepositSlidePosition.tolerance);
        depositSlide.setTargetPosition(DepositSlidePosition.specimenWall);
        depositSlide.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        depositSlide.setPower(DepositSlidePower.move);
    }
    public void specimenBar() {
        depositSlide.setTargetPositionTolerance(DepositSlidePosition.tolerance);
        depositSlide.setTargetPosition(DepositSlidePosition.specimenBar);
        depositSlide.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        depositSlide.setPower(DepositSlidePower.move);
    }
    public void highBasket() {
        depositSlide.setTargetPositionTolerance(DepositSlidePosition.tolerance);
        depositSlide.setTargetPosition(DepositSlidePosition.highBasket);
        depositSlide.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        depositSlide.setPower(DepositSlidePower.move);
    }
    public void lowBasket() {
        depositSlide.setTargetPositionTolerance(DepositSlidePosition.tolerance);
        depositSlide.setTargetPosition(DepositSlidePosition.lowBasket);
        depositSlide.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        depositSlide.setPower(DepositSlidePower.move);
    }

    public void specimenGrab() {
        try {
            claw.setPosition(Claw.ClawPosition.halfOpen);
            Thread.sleep(500);
            specimenWall();
            while (!(depositSlide.getCurrentPosition() > (DepositSlidePosition.specimenWall - DepositSlidePosition.stepRange))) {
                Thread.sleep(10);
            }
            clawArm.setPosition(Claw.ClawPosition.upClip);
            clawAngle.setPosition(Claw.ClawPosition.horizontal);
            Thread.sleep(750);
            retract();
            depositSlide.setPower(DepositSlidePower.move/2);
            claw.setPosition(Claw.ClawPosition.open);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
    }
    public void specimenHang() {
        try {
            specimenBar();
            while (!(depositSlide.getCurrentPosition() > (DepositSlidePosition.specimenBar - DepositSlidePosition.stepRange))) {
                Thread.sleep(10);
            }
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
    }
    public void depositHigh() {
        try {
            highBasket();
            while (!(depositSlide.getCurrentPosition() > (DepositSlidePosition.highBasket - DepositSlidePosition.stepRange))) {
                Thread.sleep(10);
            }
            clawArm.setPosition(Claw.ClawPosition.back);
            clawAngle.setPosition(Claw.ClawPosition.horizontal);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
    }
    public void depositLow() {
        try {
            lowBasket();
            while (!(depositSlide.getCurrentPosition() > (DepositSlidePosition.lowBasket - DepositSlidePosition.stepRange))) {
                Thread.sleep(10);
            }
            clawArm.setPosition(Claw.ClawPosition.back);
            clawAngle.setPosition(Claw.ClawPosition.horizontal);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
    }
    public void condense() {
        try {
            if (depositSlide.getCurrentPosition() <= 700) {
                move(700);
                while (!(depositSlide.getCurrentPosition() > (700 - DepositSlidePosition.stepRange))) {
                    Thread.sleep(10);
                }
            }
            clawArm.setPosition(Claw.ClawPosition.down);
            clawAngle.setPosition(Claw.ClawPosition.vertical);
            claw.setPosition(Claw.ClawPosition.closed);
            Thread.sleep(1000);
            retract();
            while (!(depositSlide.getCurrentPosition() < (DepositSlidePosition.retracted + 700))) {
                Thread.sleep(10);
            }
            depositSlide.setPower(DepositSlidePower.move/4);
            while (!(depositSlide.getCurrentPosition() < (DepositSlidePosition.retracted + DepositSlidePosition.stepRange))) {
                Thread.sleep(10);
            }
            claw.setPosition(Claw.ClawPosition.open);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
    }
}
