package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;

public class Intake extends Robot.HardwareDevices {
    @Config
    public static final class position {
        public static final double up = 0.5;
        public static final double down = 0.1;
        public static final double back = 0.1;
        public static final double center = 0.6225;
        public static final double upCenter = 0.6265;
    }
    @Config
    public static final class power {
        public static final double spinIn = 1;
        public static final double spinOut = -1;
        public static final double stop = 0;
    }

    public void yaw(double position) {
        intakeYaw.setPosition(position);
    }

    public void up() {
        intakePitch.setPosition(position.up);
        intakeYaw.setPosition(position.upCenter);
    }
    public void down() {
        intakePitch.setPosition(position.down);
    }
    public void back() {
        intakePitch.setPosition(position.back);
    }
    public void center() {
        intakeYaw.setPosition(position.center);
    }

    public void spinIn() {
        intakeRoller.setPower(power.spinIn);
    }
    public void spinOut() {
        intakeRoller.setPower(power.spinOut);
    }
    public void spinStop() {
        intakeRoller.setPower(power.stop);
    }
}
