package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;

public class DepositClaw extends Robot.HardwareDevices {

    @Config
    public static class DepositClawPosition {
        public static class Arm {
            public static double upClip = 0.5875;
            public static double upLift = 0.5625;
            public static double down = 0.66;
            public static double back = 0.4075;
            public static double forwards = 0.6475;
        }

        public static class Elbow {
            public static double vertical = 0.13;
            public static double horizontal = 0.5;
        }

        public static class Hand {
            public static double open = 0.65;
            public static double closed = 0.3;
            public static double halfOpen = 0.55;
        }
    }


    public class Arm{
        public void upClip() {
            clawArm.setPosition(DepositClawPosition.Arm.upClip);
        }
        public void upLift() {
            clawArm.setPosition(DepositClawPosition.Arm.upLift);
        }
        public void down() {
            clawArm.setPosition(DepositClawPosition.Arm.down);
        }
        public void back() {
            clawArm.setPosition(DepositClawPosition.Arm.back);
        }
        public void forwards() {
            clawArm.setPosition(DepositClawPosition.Arm.forwards);
        }
    }

    public class Elbow {
        public void vertical() {
            clawAngle.setPosition(DepositClawPosition.Elbow.vertical);
        }
        public void horizontal() {
            clawAngle.setPosition(DepositClawPosition.Elbow.horizontal);
        }
    }

    public class Hand {
        public void open() {
            claw.setPosition(DepositClawPosition.Hand.open);
        }
        public void close() {
            claw.setPosition(DepositClawPosition.Hand.closed);
        }
        public void halfOpen() {
            claw.setPosition(DepositClawPosition.Hand.halfOpen);
        }
    }

    public Arm arm = new Arm();
    public Elbow elbow = new Elbow();
    public Hand hand = new Hand();
}
