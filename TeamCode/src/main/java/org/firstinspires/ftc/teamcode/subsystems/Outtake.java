package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;

public class Outtake extends Robot.HardwareDevices {
    @Config
    public static class OuttakePositions {
        // Arm positions
        public static double armUpClip = 0.5875;
        public static double armUpLift = 0.5625;
        public static double armTransfer = 0.66;
        public static double armBack = 0.4075;

        // Wrist positions
        public static double wristVertical = 0.13;
        public static double wristHorizontal = 0.5;

        // Hand positions
        public static double handOpen = 0.65;
        public static double handClosed = 0.3;
    }

    public class Arm {
        public void upClip() {
            depositClawArmBottom.setPosition(OuttakePositions.armUpClip);
            depositClawArmTop.setPosition(OuttakePositions.armUpClip);
        }
        public void upLift() {
            depositClawArmBottom.setPosition(OuttakePositions.armUpLift);
            depositClawArmTop.setPosition(OuttakePositions.armUpLift);
        }
        public void down() {
            depositClawArmBottom.setPosition(OuttakePositions.armTransfer);
            depositClawArmTop.setPosition(OuttakePositions.armTransfer);
        }
        public void back() {
            depositClawArmBottom.setPosition(OuttakePositions.armBack);
            depositClawArmTop.setPosition(OuttakePositions.armBack);
        }
    }

    public class Wrist {
        public void vertical() {
            depositClawAngle.setPosition(OuttakePositions.wristVertical);
        }
        public void horizontal() {
            depositClawAngle.setPosition(OuttakePositions.wristHorizontal);
        }
    }

    public class Hand {
        public void open() {
            depositClaw.setPosition(OuttakePositions.handOpen);
        }
        public void close() {
            depositClaw.setPosition(OuttakePositions.handClosed);
        }
    }

    public Arm arm = new Arm();
    public Wrist wrist = new Wrist();
    public Hand hand = new Hand();
}
