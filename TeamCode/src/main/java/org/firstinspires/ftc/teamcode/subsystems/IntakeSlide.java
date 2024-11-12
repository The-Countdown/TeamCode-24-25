package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class IntakeSlide extends Robot.HardwareDevices {
    private Robot robot;

    public IntakeSlide(Robot robot) {
        this.robot = robot;
    }

    @Config
    public static class IntakeSlidePosition {
        public static int retracted = 0;
        public static int extended = 1000;
        public static int handOff = 1150;
        public static int ground = 300;
        public static int minimum = 0;
        public static int maximum = 1500;
        public static int tolerance = 5;
        public static int stepRange = 100;
    }
    @Config
    public static class IntakeSlidePower {
        public static double stop = 0;
        public static double move = 1;
    }
    public void stop() {
        intakeSlideL.setPower(IntakeSlidePower.stop);
        intakeSlideR.setPower(IntakeSlidePower.stop);
    }

    public void move(int amount){
        if (amount < IntakeSlidePosition.minimum) {
            amount = IntakeSlidePosition.minimum;
        } else if (amount > IntakeSlidePosition.maximum) {
            amount = IntakeSlidePosition.maximum;
        }

        intakeSlideL.setTargetPositionTolerance(IntakeSlidePosition.tolerance);
        intakeSlideR.setTargetPositionTolerance(IntakeSlidePosition.tolerance);
        intakeSlideL.setTargetPosition(amount);
        intakeSlideR.setTargetPosition(amount);
        intakeSlideL.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        intakeSlideR.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        intakeSlideL.setPower(IntakeSlidePower.move);
        intakeSlideR.setPower(IntakeSlidePower.move);
    }
    public void retract() {
        intakeSlideL.setTargetPositionTolerance(IntakeSlidePosition.tolerance);
        intakeSlideR.setTargetPositionTolerance(IntakeSlidePosition.tolerance);
        intakeSlideL.setTargetPosition(IntakeSlidePosition.retracted);
        intakeSlideR.setTargetPosition(IntakeSlidePosition.retracted);
        intakeSlideL.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        intakeSlideR.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        intakeSlideL.setPower(IntakeSlidePower.move);
        intakeSlideR.setPower(IntakeSlidePower.move);
    }
    public void extend() {
        intakeSlideL.setTargetPositionTolerance(IntakeSlidePosition.tolerance);
        intakeSlideR.setTargetPositionTolerance(IntakeSlidePosition.tolerance);
        intakeSlideL.setTargetPosition(IntakeSlidePosition.extended);
        intakeSlideR.setTargetPosition(IntakeSlidePosition.extended);
        intakeSlideL.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        intakeSlideR.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        intakeSlideL.setPower(IntakeSlidePower.move);
        intakeSlideR.setPower(IntakeSlidePower.move);
    }
    public void handOff() {
        intakeSlideL.setTargetPositionTolerance(IntakeSlidePosition.tolerance);
        intakeSlideR.setTargetPositionTolerance(IntakeSlidePosition.tolerance);
        intakeSlideL.setTargetPosition(IntakeSlidePosition.handOff);
        intakeSlideR.setTargetPosition(IntakeSlidePosition.handOff);
        intakeSlideL.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        intakeSlideR.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        intakeSlideL.setPower(IntakeSlidePower.move);
        intakeSlideR.setPower(IntakeSlidePower.move);
    }
    public void ground() {
        intakeSlideL.setTargetPositionTolerance(IntakeSlidePosition.tolerance);
        intakeSlideR.setTargetPositionTolerance(IntakeSlidePosition.tolerance);
        intakeSlideL.setTargetPosition(IntakeSlidePosition.ground);
        intakeSlideR.setTargetPosition(IntakeSlidePosition.ground);
        intakeSlideL.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        intakeSlideR.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        intakeSlideL.setPower(IntakeSlidePower.move);
        intakeSlideR.setPower(IntakeSlidePower.move);
    }

    public void greatHandOff() {
        try {
            robot.intakeSlide.handOff();
            while (!(((intakeSlideL.getCurrentPosition() + intakeSlideR.getCurrentPosition()) / 2) > (IntakeSlidePosition.handOff - IntakeSlidePosition.stepRange) && (((intakeSlideL.getCurrentPosition() + intakeSlideR.getCurrentPosition()) / 2) < (IntakeSlidePosition.handOff + IntakeSlidePosition.stepRange)))) {
                Thread.sleep(10);
            }
            robot.intake.wrist.horizontal();
            robot.intake.up();
            robot.outtake.arm.transfer();
            robot.outtake.hand.open();
            robot.outtake.wrist.horizontal();
            Thread.sleep(750);
            robot.intake.transferPrep();
            robot.intake.arm.transfer2();
            robot.intake.elbow.transfer();
            Thread.sleep(1150);
            robot.intake.transfer();
            Thread.sleep(300);
            robot.outtake.hand.close();
            Thread.sleep(200);
            robot.intake.hand.open();
            robot.intake.up();
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
    }
    public void pickUpGround() {
        try {
            while (!(((intakeSlideL.getCurrentPosition() + intakeSlideR.getCurrentPosition()) / 2) > (IntakeSlidePosition.extended - IntakeSlidePosition.stepRange))) {
                Thread.sleep(10);
            }
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
    }
    public void pickUp() {
        robot.intake.hand.open();
        robot.intake.down();
    }
    public void condense() {
        robot.intakeSlide.retract();
        robot.intake.rest();
    }
}