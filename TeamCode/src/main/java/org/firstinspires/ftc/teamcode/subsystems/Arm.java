package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class Arm extends Robot.HardwareDevices {
    @Config
    public static final class position {
        public static final int retracted = 0;
        public static final int extended = 1000;
        public static final int minimum = 0;
        public static final int maximum = 1000;
    }
    @Config
    public static final class power {
        public static final double stop = 0;
        public static final double move = 1;
    }

    public void stop() {
        arm.setPower(power.stop);
    }
    public void move(int amount) {
        if (amount < position.minimum) {
            amount = position.minimum;
        } else if (amount > position.maximum) {
            amount = position.maximum;
        }

        arm.setTargetPosition(amount);
        arm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        arm.setPower(power.move);
    }
    public void retract() {
        arm.setTargetPosition(position.retracted);
        arm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        arm.setPower(power.move);
    }
    public void extend() {
        arm.setTargetPosition(position.extended);
        arm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        arm.setPower(power.move);
    }
}
