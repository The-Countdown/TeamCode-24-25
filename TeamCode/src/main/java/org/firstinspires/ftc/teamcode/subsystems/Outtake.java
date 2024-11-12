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
        public static double armUpClip = 0.4025;
        public static double armUpLift = 0.39;
        public static double armTransfer = 0.39;
        public static double armBack = 0.23;
        public static double armRest = 0.51;

        // Wrist positions
        public static double wristVertical = 0.175;
        public static double wristHorizontal = 0.53;

        // Hand positions
        public static double handOpen = 0.25;
        public static double handClosed = 0.55;
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
    }

    public class Hand {
        public void open() {
            depositClaw.setPosition(OuttakePositions.handOpen);
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
