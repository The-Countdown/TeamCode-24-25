package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class DepositSlide extends Robot.HardwareDevices {
    private Robot robot;

    public DepositSlide(Robot robot) {
        this.robot = robot;
    }

    @Config
    public static class DepositSlidePosition {
        public static int retracted = 0;
        public static int highBasket = 2550;
        public static int lowBasket = 1500;
        public static int specimenWall = 700;
        public static int specimenBar = 780;
        public static int specimenBarClip = 1160;
        public static int specimenBarAltUp = 1336;
        public static int specimenBarAltDown = 1240;
        public static int transferUp = 1475;
        public static int transferDown = 1350;
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
    public void specimenBarClip() {
        depositSlide.setTargetPositionTolerance(DepositSlidePosition.tolerance);
        depositSlide.setTargetPosition(DepositSlidePosition.specimenBarClip);
        depositSlide.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        depositSlide.setPower(DepositSlidePower.move);
    }
    public void specimenBarAltUp() {
        depositSlide.setTargetPositionTolerance(DepositSlidePosition.tolerance);
        depositSlide.setTargetPosition(DepositSlidePosition.specimenBarAltUp);
        depositSlide.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        depositSlide.setPower(DepositSlidePower.move);
    }
    public void specimenBarAltDown() {
        depositSlide.setTargetPositionTolerance(DepositSlidePosition.tolerance);
        depositSlide.setTargetPosition(DepositSlidePosition.specimenBarAltDown);
        depositSlide.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        depositSlide.setPower(DepositSlidePower.move);
    }
    public void transferUp() {
        depositSlide.setTargetPositionTolerance(DepositSlidePosition.tolerance);
        depositSlide.setTargetPosition(DepositSlidePosition.transferUp);
        depositSlide.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        depositSlide.setPower(DepositSlidePower.move);
    }
    public void transferDown() {
        depositSlide.setTargetPositionTolerance(DepositSlidePosition.tolerance);
        depositSlide.setTargetPosition(DepositSlidePosition.transferDown);
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

    public void magRetract() {
        depositSlide.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        depositSlide.setPower(-DepositSlidePower.move);
        while (!Robot.HardwareDevices.depositMagnet.isPressed()) {}
        depositSlide.setPower(DepositSlidePower.stop);
        depositSlide.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        depositSlide.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        depositSlide.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
    }

    public void specimenGrab() {
        try {
            robot.intake.wrist.horizontal();
            robot.intake.hand.open();
            robot.intakeSlide.retract();
            robot.intake.elbow.up();
            specimenWall();
            Thread.sleep(600);
            robot.intake.arm.rest();
            robot.intake.elbow.rest();
            while (!(depositSlide.getCurrentPosition() > (DepositSlidePosition.specimenWall - DepositSlidePosition.stepRange))) {}
            robot.outtake.arm.upClip();
            robot.outtake.wrist.horizontal();
            Thread.sleep(750);
            magRetract();
            robot.outtake.hand.open();
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
    }
    public void specimenHang() {
        try {
            specimenBar();
            robot.outtake.arm.back();
            while (!(depositSlide.getCurrentPosition() > (DepositSlidePosition.specimenBar - DepositSlidePosition.stepRange))) {
                Thread.sleep(10);
            }
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
    }
    public void actTwo() {
        try {
            robot.intake.elbow.transfer();
            Thread.sleep(1000);
            transferDown();
            Robot.HardwareDevices.depositSlide.setPower(DepositSlidePower.move/3);
            while (!(depositSlide.getCurrentPosition() < (DepositSlidePosition.transferDown + 15))) {
                Thread.sleep(10);
            }
            robot.outtake.hand.close();
            Thread.sleep(250);
            robot.intake.hand.open();
            Thread.sleep(250);
            highBasket();
            Robot.HardwareDevices.depositSlide.setPower(DepositSlidePower.move/2);
            robot.outtake.arm.upClip();
            Thread.sleep(400);
            Robot.HardwareDevices.depositSlide.setPower(DepositSlidePower.move);
            while (!(depositSlide.getCurrentPosition() > (DepositSlidePosition.highBasket - 300))) {
                Thread.sleep(10);
            }
            robot.intake.arm.up();
            robot.intake.wrist.horizontal();
            robot.intake.elbow.down();
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
    public void condensedMilk() {
        try {
            if (depositSlide.getCurrentPosition() <= 1200) {
                move(1200);
                while (!(depositSlide.getCurrentPosition() > (1200 - DepositSlidePosition.stepRange))) {
                }
            }
            robot.outtake.arm.rest();
            robot.outtake.wrist.vertical();
            robot.outtake.hand.close();
            Thread.sleep(1000);
            retract();
            while (!(depositSlide.getCurrentPosition() < 500)) {
            }
            depositSlide.setPower(DepositSlidePower.move / 4);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
    }
}
