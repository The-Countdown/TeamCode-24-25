package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class DepositSlide extends Robot.HardwareDevices {
    private Robot robot;

    public DepositSlide(Robot robot) {
        this.robot = robot;
    }

    @Config
    public static class DepositSlidePosition {
        public static int retracted = 0;
        public static int highBasket = 2500;
        public static int lowBasket = 1500;
        public static int specimenWall = 700;
        public static int specimenBar = 1200;
        public static int tolerance = 5;
        public static int stepRange = 50;
        public static int stopTolerance = 10;
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
            robot.intakeSlide.condense();
            specimenWall();
            while (!(depositSlide.getCurrentPosition() > (DepositSlidePosition.specimenWall - DepositSlidePosition.stepRange))) {
                Thread.sleep(10);
            }
            depositSlide.setPower(DepositSlidePower.move/2);
            robot.outtake.arm.upClip();
            robot.outtake.wrist.horizontal();
            Thread.sleep(750);
            retract();
            intakeSlideL.setPower(IntakeSlide.IntakeSlidePower.move/2);
            intakeSlideR.setPower(IntakeSlide.IntakeSlidePower.move/2);
            robot.outtake.hand.open();
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
            robot.outtake.hand.close();
            highBasket();
            robot.intakeSlide.retract();
            while (!(depositSlide.getCurrentPosition() > (DepositSlidePosition.highBasket - 300))) {
                Thread.sleep(10);
            }
            robot.outtake.arm.back();
            robot.outtake.wrist.horizontal();
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
    }
    public void depositLow() {
        try {
            robot.outtake.hand.close();
            lowBasket();
            robot.intakeSlide.retract();
            while (!(depositSlide.getCurrentPosition() > (DepositSlidePosition.lowBasket - 200))) {
                Thread.sleep(10);
            }
            robot.outtake.arm.back();
            robot.outtake.wrist.horizontal();
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
    }
    public void condense() {
        try {
            if (depositSlide.getCurrentPosition() <= 800) {
                move(800);
                while (!(depositSlide.getCurrentPosition() > (800 - DepositSlidePosition.stepRange))) {
                    Thread.sleep(10);
                }
            }
            robot.outtake.arm.rest();
            robot.outtake.wrist.vertical();
            robot.outtake.hand.close();
            Thread.sleep(1000);
            retract();
            while (!(depositSlide.getCurrentPosition() < 500)) {
                Thread.sleep(10);
            }
            depositSlide.setPower(DepositSlidePower.move / 4);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
    }
}
