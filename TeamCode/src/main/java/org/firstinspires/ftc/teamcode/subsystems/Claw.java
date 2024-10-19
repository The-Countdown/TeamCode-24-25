package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;

public class Claw extends Robot.HardwareDevices {
    @Config
    public static final class position {
        public static final double open = 0;
        public static final double closed = 0.5;
        public static final double vertical = 0.625;
        public static final double horizontal = 0.27;
        public static final double up = 0.575;
        public static final double down = 0.6075;
        public static final double back = 0.42;
        public static final double forwards = 0.5925;
    }

    public void open() {
        claw.setPosition(position.open);
    }
    public void close() {
        claw.setPosition(position.closed);
    }
    public void vertical() {
        clawAngle.setPosition(position.vertical);
    }
    public void horizontal() {
        clawAngle.setPosition(position.horizontal);
    }
    public void up() {
        clawArm.setPosition(position.up);
    }
    public void down() {
        clawArm.setPosition(position.down);
    }
    public void back() {
        clawArm.setPosition(position.back);
    }
    public void forwards() {
        clawArm.setPosition(position.forwards);
    }
}
