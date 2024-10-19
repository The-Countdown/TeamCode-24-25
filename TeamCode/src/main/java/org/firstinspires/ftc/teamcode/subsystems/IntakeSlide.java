package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class IntakeSlide extends Robot.HardwareDevices {
    @Config
    public static final class position {
        public static final int retracted = 0;
        public static final int extended = 1500;
        public static final int minimum = 0;
        public static final int maximum = 1500;
        public static final int tolerance = 5;
    }
    @Config
    public static final class power {
        public static final double stop = 0;
        public static final double move = 1;
    }
    public void stop() {
        intakeSlide.setPower(power.stop);
    }

    public void move(int amount){
        if (amount < position.minimum) {
            amount = position.minimum;
        } else if (amount > position.maximum) {
            amount = position.maximum;
        }

        intakeSlide.setTargetPosition(amount);
        intakeSlide.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        intakeSlide.setPower(power.move);
    }
    public void retract() {
        intakeSlide.setTargetPosition(position.retracted);
        intakeSlide.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        intakeSlide.setPower(power.move);
    }
    public void extend() {
        intakeSlide.setTargetPosition(position.extended);
        intakeSlide.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        intakeSlide.setPower(power.move);
    }
    public void pickUp() {
        try {
            intakePitch.setPosition(Intake.position.up);
            intakeYaw.setPosition((Intake.position.center) + 0.004);
            Thread.sleep(750);
            intakeSlide.setTargetPositionTolerance(5);
            intakeSlide.setTargetPosition(position.extended);
            intakeSlide.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            intakeSlide.setPower(power.move);
            while (!(intakeSlide.getCurrentPosition() > (position.extended - 100))) {
                Thread.sleep(10);
            }
            intakeSlide.setPower(power.move / position.tolerance);
            intakePitch.setPosition(Intake.position.down);
            intakeYaw.setPosition((Intake.position.center));
            intakeRoller.setPower(Intake.power.spinIn);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
    }
    public void condense() {
        try {
            intakePitch.setPosition(Intake.position.up);
            intakeYaw.setPosition((Intake.position.center) + 0.003);
            Thread.sleep(750);
            intakeSlide.setTargetPositionTolerance(position.tolerance);
            intakeSlide.setTargetPosition(position.retracted);
            intakeSlide.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            intakeSlide.setPower(power.move);
            while (!(intakeSlide.getCurrentPosition() > -100)) {
                Thread.sleep(10);
            }
            intakePitch.setPosition(Intake.position.down);
            intakeYaw.setPosition((Intake.position.center));
            intakeRoller.setPower(Intake.power.stop);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
    }
}