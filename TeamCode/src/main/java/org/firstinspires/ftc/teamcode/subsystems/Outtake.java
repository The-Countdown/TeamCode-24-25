package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;

public class Outtake extends Robot.HardwareDevices {
    private Robot robot;

    public Outtake(Robot robot) {
        this.robot = robot;
    }

    @Config
    public static class OuttakePositions {
        // Arm positions
        public static double armUpClip = 0.31;
        public static double armUpLift = 0.5;
        public static double armTransfer = 0.03;
        public static double armBack = 0.97;
        public static double armRest = 0;

        // Wrist positions
        public static double wristVertical = 0.38;
        public static double wristHorizontal = 0.02;
        public static double wristHorizontalFlip = 0.745;

        // Hand positions
        public static double handOpen = 0.35;
        public static double handClosed = 0.7;
        public static double handHalfOpen = 0.53;
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
        public void transfer() {
            depositClawArmBottom.setPosition(OuttakePositions.armTransfer);
            depositClawArmTop.setPosition(OuttakePositions.armTransfer);
        }
        public void back() {
            depositClawArmBottom.setPosition(OuttakePositions.armBack);
            depositClawArmTop.setPosition(OuttakePositions.armBack);
        }
        public void rest() {
            depositClawArmBottom.setPosition(OuttakePositions.armRest);
            depositClawArmTop.setPosition(OuttakePositions.armRest);
        }
    }

    public class Wrist {
        public void vertical() {
            depositClawAngle.setPosition(OuttakePositions.wristVertical);
        }
        public void horizontal() {
            depositClawAngle.setPosition(OuttakePositions.wristHorizontal);
        }
        public void horizontalFlip() {
            depositClawAngle.setPosition(OuttakePositions.wristHorizontalFlip);
        }
    }

    public class Hand {
        public void open() {
            depositClaw.setPosition(OuttakePositions.handOpen);
        }
        public void halfOpen() {
            depositClaw.setPosition(OuttakePositions.handHalfOpen);
        }
        public void close() {
            depositClaw.setPosition(OuttakePositions.handClosed);
        }
    }

    public void rest() {
        robot.outtake.wrist.vertical();
        robot.outtake.hand.close();
        robot.outtake.arm.rest();
    }

    public Arm arm = new Arm();
    public Wrist wrist = new Wrist();
    public Hand hand = new Hand();
}
