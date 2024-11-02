package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.subsystems.Robot.rb;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class IntakeSlide extends Robot.HardwareDevices {
    @Config
    public static class IntakeSlidePosition {
        public static int retracted = 0;
        public static int extended = 1000;
        public static int ground = 300; //TODO: Find
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
    public void pickUpGround() {
        try {
            clawArm.setPosition(Claw.ClawPosition.down);
            claw.setPosition(Claw.ClawPosition.open);
            Thread.sleep(250);
            ground();
            while (!(((intakeSlideL.getCurrentPosition() + intakeSlideR.getCurrentPosition()) / 2) > (IntakeSlidePosition.ground - IntakeSlidePosition.stepRange))) {
                Thread.sleep(10);
            }
            intakePitchL.setPosition(Intake.IntakePosition.downL);
            intakePitchR.setPosition(Intake.IntakePosition.downR);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
    }
    public void pickUp() {
        try {
            rb.depositSlide.retract();
            clawArm.setPosition(Claw.ClawPosition.down);
            claw.setPosition(Claw.ClawPosition.open);
            Thread.sleep(250);
            intakePitchL.setPosition(Intake.IntakePosition.upL);
            intakePitchR.setPosition(Intake.IntakePosition.upR);
            Thread.sleep(750);
            extend();
            while (!(((intakeSlideL.getCurrentPosition() + intakeSlideR.getCurrentPosition()) / 2) > (IntakeSlidePosition.extended - IntakeSlidePosition.stepRange))) {
                Thread.sleep(10);
            }
            intakeSlideL.setPower(IntakeSlidePower.move / IntakeSlidePosition.tolerance);
            intakeSlideR.setPower(IntakeSlidePower.move / IntakeSlidePosition.tolerance);
            intakePitchL.setPosition(Intake.IntakePosition.downL);
            intakePitchR.setPosition(Intake.IntakePosition.downR);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
    }
    public void condense() {
        try {
            clawArm.setPosition(Claw.ClawPosition.down);
            claw.setPosition(Claw.ClawPosition.open);
            intakePitchL.setPosition(Intake.IntakePosition.upL);
            intakePitchR.setPosition(Intake.IntakePosition.upR);
            Thread.sleep(750);
            retract();
            while (!(((intakeSlideL.getCurrentPosition() + intakeSlideR.getCurrentPosition()) / 2) < 200)) {
                Thread.sleep(10);
            }
            intakePitchL.setPosition(Intake.IntakePosition.downL - 0.04);
            intakePitchR.setPosition(Intake.IntakePosition.downR + 0.04);
            Thread.sleep(500);
            intakePitchL.setPosition(Intake.IntakePosition.downL - 0.005);
            intakePitchR.setPosition(Intake.IntakePosition.downR + 0.005);
            Thread.sleep(500);
            claw.setPosition(Claw.ClawPosition.closed);
            Thread.sleep(500);
            clawArm.setPosition(Claw.ClawPosition.forwards);
            depositSlide.setTargetPositionTolerance(3);
            depositSlide.setTargetPosition(DepositSlide.DepositSlidePosition.retracted);
            depositSlide.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            depositSlide.setPower(DepositSlide.DepositSlidePower.move);
            Thread.sleep(750);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
    }
}