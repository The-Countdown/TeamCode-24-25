package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;

public class Intake extends Robot.HardwareDevices {
    @Config
    public static class IntakePosition {
        public static double upL = 0.52;
        public static double upR = 0.63;
        public static double downL = 0.61;
        public static double downR = 0.54;
        public static double alignL = 0.58;
        public static double alignR = 0.57;
        public static double back = 0.1;
        public static double center = 0.615;
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
        intakePitchL.setPosition(IntakePosition.upL);
        intakePitchR.setPosition(IntakePosition.upR);
    }
    public void down() {
        intakePitchL.setPosition(IntakePosition.downL);
        intakePitchR.setPosition(IntakePosition.downR);
    }
    public void align() {
        intakePitchL.setPosition(IntakePosition.alignL);
        intakePitchR.setPosition(IntakePosition.alignR);
    }
    public void back() {
        intakePitchL.setPosition(IntakePosition.back);
        intakePitchR.setPosition(IntakePosition.back);
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
