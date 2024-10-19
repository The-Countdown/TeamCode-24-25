package org.firstinspires.ftc.teamcode.subsystems;


import static org.firstinspires.ftc.teamcode.OldTeleOp.Teleoperation.intakeExtended;
import static org.firstinspires.ftc.teamcode.OldTeleOp.Teleoperation.intakePosDown;
import static org.firstinspires.ftc.teamcode.OldTeleOp.Teleoperation.intakePosUp;
import static org.firstinspires.ftc.teamcode.OldTeleOp.Teleoperation.intakePower;
import static org.firstinspires.ftc.teamcode.OldTeleOp.Teleoperation.intakeRetracted;
import static org.firstinspires.ftc.teamcode.OldTeleOp.Teleoperation.intakeRollerSpeed;
import static org.firstinspires.ftc.teamcode.OldTeleOp.Teleoperation.intakeYawCenter;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class IntakeSlide extends Robot.HardwareDevices {
    @Config
    public static final class position {
        public static final int retracted = 0;
        public static final int extended = 1500;
        public static final int minimum = 0;
        public static final int maximum = 1500;
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
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
    }
    public void condense() {
        try {
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
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
    }
}