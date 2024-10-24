package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;

public class Claw extends Robot.HardwareDevices {
    @Config
    public static class ClawPosition {
        public static double open = 0;
        public static double closed = 0.53;
        public static double vertical = 0.625;
        public static double horizontal = 0.27;
        public static double up = 0.555;
        public static double down = 0.6425;
        public static double back = 0.4;
        public static double forwards = 0.635;
    }

    public void open() {
        claw.setPosition(ClawPosition.open);
    }
    public void close() {
        claw.setPosition(ClawPosition.closed);
    }
    public void vertical() {
        clawAngle.setPosition(ClawPosition.vertical);
    }
    public void horizontal() {
        clawAngle.setPosition(ClawPosition.horizontal);
    }
    public void up() {
        clawArm.setPosition(ClawPosition.up);
    }
    public void down() {
        clawArm.setPosition(ClawPosition.down);
    }
    public void back() {
        clawArm.setPosition(ClawPosition.back);
    }
    public void forwards() {
        clawArm.setPosition(ClawPosition.forwards);
    }
}
