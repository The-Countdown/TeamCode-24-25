package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.subsystems.Robot.rb;

import com.acmerobotics.dashboard.config.Config;

public class Intake extends Robot.HardwareDevices {
    @Config
    public static class IntakePosition {
        // Arm positions
        public static double armUpL = 0;
        public static double armUpR = 0;
        public static double armDownL = 0;
        public static double armDownR = 0;
        public static double armRestL = 0;
        public static double armRestR = 0;
        public static double armTransferL = 0;
        public static double armTransferR = 0;

        // Elbow positions
        public static double elbowUp = 0;
        public static double elbowRest = 0;
        public static double elbowDown = 0;
        public static double elbowTransfer = 0;

        // Wrist positions
        public static double wristVertical = 0;
        public static double wristHorizontal = 0;

        // Hand positions
        public static double handOpen = 0;
        public static double handClosed = 0;
        public static double handHalfOpen = 0;
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
        public void closed() {
            intakeClaw.setPosition(IntakePosition.handClosed);
        }
        public void halfOpen() {
            intakeClaw.setPosition(IntakePosition.handHalfOpen);
        }
    }

    public void up() {
        rb.intake.arm.up();
        rb.intake.elbow.down();
    }

    public void down() {
        rb.intake.arm.down();
        rb.intake.elbow.down();
    }

    public void rest() {
        rb.intake.arm.rest();
        rb.intake.elbow.rest();
        rb.intake.wrist.horizontal();
    }

    public void transfer() {
        rb.intake.arm.transfer();
        rb.intake.elbow.transfer();
        rb.intake.wrist.vertical();
    }

    public void transferPrep() {
        try {
            rb.intake.wrist.vertical();
            rb.intake.elbow.up();
            Thread.sleep(500);
            rb.intake.hand.halfOpen();
            Thread.sleep(300);
            rb.intake.hand.open();
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
    }

    public Arm arm = new Arm();
    public Elbow elbow = new Elbow();
    public Wrist wrist = new Wrist();
    public Hand hand = new Hand();
}
