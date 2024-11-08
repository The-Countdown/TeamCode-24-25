package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;

public class Outtake extends Robot.HardwareDevices {

    @Config
    public static class OuttakePositions {
        public static class Arm {
            public static double upClip = 0.5875;
            public static double upLift = 0.5625;
            public static double transfer = 0.66;
            public static double back = 0.4075;
        }

        public static class Wrist {
            public static double vertical = 0.13;
            public static double horizontal = 0.5;
        }

        public static class Hand {
            public static double open = 0.65;
            public static double closed = 0.3;
        }
    }


    public class Arm{
        public void upClip() {
            depositClawArmBottom.setPosition(OuttakePositions.Arm.upClip);
            depositClawArmTop.setPosition(OuttakePositions.Arm.upClip);
        }
        public void upLift() {
            depositClawArmBottom.setPosition(OuttakePositions.Arm.upLift);
            depositClawArmTop.setPosition(OuttakePositions.Arm.upLift);
        }
        public void down() {
            depositClawArmBottom.setPosition(OuttakePositions.Arm.transfer);
            depositClawArmTop.setPosition(OuttakePositions.Arm.transfer);
        }
        public void back() {
            depositClawArmBottom.setPosition(OuttakePositions.Arm.back);
            depositClawArmTop.setPosition(OuttakePositions.Arm.back);
        }
    }

    public class Wrist {
        public void vertical() {
            depositClawAngle.setPosition(OuttakePositions.Wrist.vertical);
        }
        public void horizontal() {
            depositClawAngle.setPosition(OuttakePositions.Wrist.horizontal);
        }
    }

    public class Hand {
        public void open() {
            depositClaw.setPosition(OuttakePositions.Hand.open);
        }
        public void close() {
            depositClaw.setPosition(OuttakePositions.Hand.closed);
        }
    }

    public Arm arm = new Arm();
    public Wrist wrist = new Wrist();
    public Hand hand = new Hand();
}
