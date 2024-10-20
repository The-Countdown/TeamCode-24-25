package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class Arm extends Robot.HardwareDevices {
    @Config
    public static class ArmPosition {
        public static int retracted = 0;
        public static int extended = 1000;
        public static int minimum = 0;
        public static int maximum = 1000;
    }
    @Config
    public static class ArmPower {
        public static double stop = 0;
        public static double move = 1;
    }

    public void stop() {
        arm.setPower(ArmPower.stop);
    }
    public void move(int amount) {
        if (amount < ArmPosition.minimum) {
            amount = ArmPosition.minimum;
        } else if (amount > ArmPosition.maximum) {
            amount = ArmPosition.maximum;
        }

        arm.setTargetPosition(amount);
        arm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        arm.setPower(ArmPower.move);
    }
    public void retract() {
        arm.setTargetPosition(ArmPosition.retracted);
        arm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        arm.setPower(ArmPower.move);
    }
    public void extend() {
        arm.setTargetPosition(ArmPosition.extended);
        arm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        arm.setPower(ArmPower.move);
    }
}
