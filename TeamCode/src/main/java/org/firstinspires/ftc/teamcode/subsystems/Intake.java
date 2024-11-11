package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;

public class Intake extends Robot.HardwareDevices {
    private Robot robot;

    public Intake(Robot robot) {
        this.robot = robot;
    }

    @Config
    public static class IntakePosition {
        // Arm positions
        public static double armUpL = 0.5225;
        public static double armUpR = 0.4775;
        public static double armDownL = 0.51;
        public static double armDownR = 0.49;
        public static double armRestL = 0.395;
        public static double armRestR = 0.605;
        public static double armTransferL = 0.5675;
        public static double armTransferR = 0.4325;
        public static double armTransfer2L = 0.54;
        public static double armTransfer2R = 0.46;

        // Elbow positions
        public static double elbowUp = 0.37;
        public static double elbowRest = 0.4;
        public static double elbowDown = 0.46;
        public static double elbowTransfer = 0.2075;

        // Wrist positions
        public static double wristVertical = 0.545;
        public static double wristHorizontal = 0.175;

        // Hand positions
        public static double handOpen = 0.5;
        public static double handClosed = 0.775;
        public static double handHalfOpen = 0.725;
    }

    public class Arm {
        public void up() {
            intakePitchL.setPosition(IntakePosition.armUpL);
            intakePitchR.setPosition(IntakePosition.armUpR);
        }
        public void down() {
            intakePitchL.setPosition(IntakePosition.armDownL);
            intakePitchR.setPosition(IntakePosition.armDownR);
        }
        public void rest() {
            intakePitchL.setPosition(IntakePosition.armRestL);
            intakePitchR.setPosition(IntakePosition.armRestR);
        }
        public void transfer() {
            intakePitchL.setPosition(IntakePosition.armTransferL);
            intakePitchR.setPosition(IntakePosition.armTransferR);
        }
        public void transfer2() {
            intakePitchL.setPosition(IntakePosition.armTransfer2L);
            intakePitchR.setPosition(IntakePosition.armTransfer2R);
        }
    }

    public class Elbow {
        public void up() {
            intakeCoaxialPitch.setPosition(IntakePosition.elbowUp);
        }
        public void down() {
            intakeCoaxialPitch.setPosition(IntakePosition.elbowDown);
        }
        public void rest() {
            intakeCoaxialPitch.setPosition(IntakePosition.elbowRest);
        }
        public void transfer() {
            intakeCoaxialPitch.setPosition(IntakePosition.elbowTransfer);
        }
    }

    public class Wrist {
        public void vertical() {
            intakeClawAngle.setPosition(IntakePosition.wristVertical);
        }
        public void horizontal() {
            intakeClawAngle.setPosition(IntakePosition.wristHorizontal);
        }
    }

    public class Hand {
        public void open() {
            intakeClaw.setPosition(IntakePosition.handOpen);
        }
        public void close() {
            intakeClaw.setPosition(IntakePosition.handClosed);
        }
        public void halfOpen() {
            intakeClaw.setPosition(IntakePosition.handHalfOpen);
        }
    }

    public void up() {
        robot.intake.arm.up();
        robot.intake.elbow.up();
    }

    public void down() {
        robot.intake.arm.down();
        robot.intake.elbow.down();
    }

    public void rest() {
        robot.intake.arm.rest();
        robot.intake.elbow.rest();
        robot.intake.wrist.horizontal();
    }

    public void transfer() {
        robot.intake.arm.transfer();
        robot.intake.elbow.transfer();
        robot.intake.wrist.horizontal();
    }

    public void transferPrep() {
        try {
            robot.intake.wrist.horizontal();
            robot.intake.elbow.up();
            robot.intake.hand.halfOpen();
            Thread.sleep(200);
            robot.intake.hand.close();
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
    }

    public void restEsc() {
        // assuming elbow and arm are in the rest position
        try {
            robot.intakeSlide.move(300);
            while (!(((intakeSlideL.getCurrentPosition() + intakeSlideR.getCurrentPosition()) / 2) > 290)) {
                Thread.sleep(5);
            }
            robot.intake.arm.up();
            robot.intake.elbow.up();
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
    }

    public Arm arm = new Arm();
    public Elbow elbow = new Elbow();
    public Wrist wrist = new Wrist();
    public Hand hand = new Hand();
}
