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
        public static double armUpL = 0.55;
        public static double armUpR = 0.45;
        public static double armDownL = 0.53;
        public static double armDownR = 0.47;
        public static double armRestL = 0.425;
        public static double armRestR = 0.575;
        public static double armTransferL = 0.6;
        public static double armTransferR = 0.4;
        public static double armTransfer2L = 0.56;
        public static double armTransfer2R = 0.44;

        // Elbow positions
        public static double elbowUp = 0.32;
        public static double elbowRest = 0.35;
        public static double elbowDown = 0.43;
        public static double elbowTransfer = 0.175;

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
        robot.intake.hand.open();
    }

    public void transfer() {
        robot.intake.arm.transfer();
        robot.intake.elbow.transfer();
        robot.intake.wrist.horizontal();
    }

    public void greatHandOff() {
        try {
            robot.intakeSlide.handOff();
            while (!(((intakeSlideL.getCurrentPosition() + intakeSlideR.getCurrentPosition()) / 2) > (IntakeSlide.IntakeSlidePosition.handOff - IntakeSlide.IntakeSlidePosition.stepRange) && (((intakeSlideL.getCurrentPosition() + intakeSlideR.getCurrentPosition()) / 2) < (IntakeSlide.IntakeSlidePosition.handOff + IntakeSlide.IntakeSlidePosition.stepRange)))) {
                Thread.sleep(10);
            }
            robot.intake.wrist.horizontal();
            robot.intake.up();
            robot.outtake.arm.transfer();
            robot.outtake.hand.open();
            robot.outtake.wrist.horizontal();
            Thread.sleep(750);
            robot.intake.transferPrep();
            robot.intake.arm.transfer2();
            robot.intake.elbow.transfer();
            Thread.sleep(1150);
            robot.intake.transfer();
            Thread.sleep(300);
            robot.outtake.hand.close();
            Thread.sleep(200);
            robot.intake.hand.open();
            robot.intake.up();
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
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
        robot.intake.arm.up();
        robot.intake.elbow.up();
    }

    public Arm arm = new Arm();
    public Elbow elbow = new Elbow();
    public Wrist wrist = new Wrist();
    public Hand hand = new Hand();
}
