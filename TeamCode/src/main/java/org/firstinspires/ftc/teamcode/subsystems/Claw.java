package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;

public class Claw extends Robot.HardwareDevices {
    @Config
    public static class ClawPosition {
        public static double open = 0.65;
        public static double closed = 0.3;
        public static double vertical = 0.13;
        public static double horizontal = 0.5;
        public static double upClip = 0.5875;
        public static double upLift = 0.5625;
        public static double down = 0.66;
        public static double back = 0.4075;
        public static double halfOpen = 0.55;
        public static double forwards = 0.6475;
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
    public void upWall() {
        clawArm.setPosition(ClawPosition.upClip);
    }
    public void upBar() {
        clawArm.setPosition(ClawPosition.upLift);
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
