package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class IntakeSlide extends Robot.HardwareDevices {
    @Config
    public static class IntakeSlidePosition {
        public static int retracted = 0;
        public static int extended = 1500;
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
        intakeSlide.setPower(IntakeSlidePower.stop);
    }

    public void move(int amount){
        if (amount < IntakeSlidePosition.minimum) {
            amount = IntakeSlidePosition.minimum;
        } else if (amount > IntakeSlidePosition.maximum) {
            amount = IntakeSlidePosition.maximum;
        }

        intakeSlide.setTargetPosition(amount);
        intakeSlide.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        intakeSlide.setPower(IntakeSlidePower.move);
    }
    public void retract() {
        intakeSlide.setTargetPosition(IntakeSlidePosition.retracted);
        intakeSlide.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        intakeSlide.setPower(IntakeSlidePower.move);
    }
    public void extend() {
        intakeSlide.setTargetPosition(IntakeSlidePosition.extended);
        intakeSlide.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        intakeSlide.setPower(IntakeSlidePower.move);
    }
    public void pickUp() {
        try {
            intakePitch.setPosition(Intake.IntakePosition.up);
            intakeYaw.setPosition((Intake.IntakePosition.upCenter));
            Thread.sleep(750);
            intakeSlide.setTargetPositionTolerance(IntakeSlidePosition.tolerance);
            intakeSlide.setTargetPosition(IntakeSlidePosition.extended);
            intakeSlide.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            intakeSlide.setPower(IntakeSlidePower.move);
            while (!(intakeSlide.getCurrentPosition() > (IntakeSlidePosition.extended - IntakeSlidePosition.stepRange))) {
                Thread.sleep(10);
            }
            intakeSlide.setPower(IntakeSlidePower.move / IntakeSlidePosition.tolerance);
            intakePitch.setPosition(Intake.IntakePosition.down);
            intakeYaw.setPosition((Intake.IntakePosition.center));
            intakeRoller.setPower(Intake.IntakePower.spinIn);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
    }
    public void condense() {
        try {
            intakePitch.setPosition(Intake.IntakePosition.up);
            intakeYaw.setPosition(Intake.IntakePosition.upCenter);
            Thread.sleep(750);
            intakeSlide.setTargetPositionTolerance(IntakeSlidePosition.tolerance);
            intakeSlide.setTargetPosition(IntakeSlidePosition.retracted);
            intakeSlide.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            intakeSlide.setPower(IntakeSlidePower.move);
            while (!(intakeSlide.getCurrentPosition() > -100)) {
                Thread.sleep(10);
            }
            intakePitch.setPosition(Intake.IntakePosition.down);
            intakeYaw.setPosition((Intake.IntakePosition.center));
            intakeRoller.setPower(Intake.IntakePower.stop);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
    }
}