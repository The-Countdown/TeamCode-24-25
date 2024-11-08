package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;

public class Intake extends Robot.HardwareDevices {
    @Config
    public static class IntakePosition { //TODO: FIND ALL
        public static class Arm {
            public static double upL = 0;
            public static double upR = 0;
            public static double downL = 0;
            public static double downR = 0;
            public static double restL = 0;
            public static double restR = 0;
            public static double transferL = 0;
            public static double transferR = 0;
        }

        public static class Elbow {
            public static double up = 0;
            public static double rest = 0;
            public static double down = 0;
            public static double transfer = 0;
        }
        public static class Wrist {
            public static double vertical = 0;
            public static double horizontal = 0;
        }
        public static class Hand {
            public static double open = 0;
            public static double closed = 0;
            public static double halfOpen = 0;
        }
    }

    public class Arm {
        public void up() {
            intakePitchL.setPosition(IntakePosition.Arm.upL);
            intakePitchR.setPosition(IntakePosition.Arm.upR);
        }
        public void down() {
            intakePitchL.setPosition(IntakePosition.Arm.downL);
            intakePitchR.setPosition(IntakePosition.Arm.downR);
        }
        public void rest() {
            intakePitchL.setPosition(IntakePosition.Arm.restL);
            intakePitchR.setPosition(IntakePosition.Arm.restR);
        }
        public void transfer() {
            intakePitchL.setPosition(IntakePosition.Arm.transferL);
            intakePitchR.setPosition(IntakePosition.Arm.transferR);
        }
    }
    public class Elbow {
        public void up() {
            intakeCoaxialPitch.setPosition(IntakePosition.Elbow.up);
        }
        public void down() {
            intakeCoaxialPitch.setPosition(IntakePosition.Elbow.down);
        }
        public void rest() {
            intakeCoaxialPitch.setPosition(IntakePosition.Elbow.rest);
        }
        public void transfer() {
            intakeCoaxialPitch.setPosition(IntakePosition.Elbow.transfer);
        }
    }
    public class Wrist {
        public void vertical() {
            intakeClawAngle.setPosition(IntakePosition.Wrist.vertical);
        }
        public void horizontal() {
            intakeClawAngle.setPosition(IntakePosition.Wrist.horizontal);
        }
    }
    public class Hand {
        public void open() {
            intakeClaw.setPosition(IntakePosition.Hand.open);
        }
        public void closed() {
            intakeClaw.setPosition(IntakePosition.Hand.closed);
        }
        public void halfOpen() {
            intakeClaw.setPosition(IntakePosition.Hand.halfOpen);
        }
    }

    public void up() {
        intakePitchL.setPosition(IntakePosition.Arm.upL);
        intakePitchR.setPosition(IntakePosition.Arm.upR);
        intakeCoaxialPitch.setPosition(IntakePosition.Elbow.down);
    }
    public void down() {
        intakePitchL.setPosition(IntakePosition.Arm.downL);
        intakePitchR.setPosition(IntakePosition.Arm.downR);
        intakeCoaxialPitch.setPosition(IntakePosition.Elbow.down);
    }
    public void rest() {
        intakePitchL.setPosition(IntakePosition.Arm.restL);
        intakePitchR.setPosition(IntakePosition.Arm.restR);
        intakeCoaxialPitch.setPosition(IntakePosition.Elbow.rest);
        intakeClawAngle.setPosition(IntakePosition.Wrist.horizontal);
    }
    public void transfer() {
        intakePitchL.setPosition(IntakePosition.Arm.transferL);
        intakePitchR.setPosition(IntakePosition.Arm.transferR);
        intakeCoaxialPitch.setPosition(IntakePosition.Elbow.transfer);
        intakeClawAngle.setPosition(IntakePosition.Wrist.vertical);
    }
    public void transferPrep() {
        try {
            intakeClawAngle.setPosition(IntakePosition.Wrist.vertical);
            intakeCoaxialPitch.setPosition(IntakePosition.Elbow.up);
            Thread.sleep(500);
            intakeClaw.setPosition(IntakePosition.Hand.halfOpen);
            Thread.sleep(300);
            intakeClaw.setPosition(IntakePosition.Hand.open);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
    }
    public Arm arm = new Arm();
    public Elbow elbow = new Elbow();
    public Wrist wrist = new Wrist();
    public Hand hand = new Hand();
}
