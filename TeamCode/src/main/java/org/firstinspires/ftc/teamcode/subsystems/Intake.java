package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;

public class Intake extends Robot.HardwareDevices {
    @Config
    public static class IntakePosition {
        public static double up = 0.5;
        public static double down = 0.6;
        public static double back = 0.1;
        public static double center = 0.615;
        public static double upCenter = 0.62;
    }
    @Config
    public static class IntakePower {
        public static double spinIn = 1;
        public static double spinOut = -1;
        public static double stop = 0;
    }

    public void yaw(double position) {
        intakeYaw.setPosition(position);
    }

    public void up() {
        intakePitch.setPosition(IntakePosition.up);
        intakeYaw.setPosition(IntakePosition.upCenter);
    }
    public void down() {
        intakePitch.setPosition(IntakePosition.down);
    }
    public void back() {
        intakePitch.setPosition(IntakePosition.back);
    }
    public void center() {
        intakeYaw.setPosition(IntakePosition.center);
    }

    public void spinIn() {
        intakeRoller.setPower(IntakePower.spinIn);
    }
    public void spinOut() {
        intakeRoller.setPower(IntakePower.spinOut);
    }
    public void spinStop() {
        intakeRoller.setPower(IntakePower.stop);
    }
}
