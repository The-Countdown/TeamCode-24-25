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
        public static double armUpL = 0.6025;
        public static double armUpR = 0.3975;
        public static double armDownL = 0.5825;
        public static double armDownR = 0.4175;
        public static double armRestL = 0.4675;
        public static double armRestR = 0.5325;
        public static double armTransferL = 0.64;
        public static double armTransferR = 0.36;
        public static double armTransfer2L = 0.6025;
        public static double armTransfer2R = 0.3975;

        // Elbow positions
        public static double elbowUp = 0.265;
        public static double elbowRest = 0.295;
        public static double elbowDown = 0.37;
        public static double elbowTransfer = 0.17;

        // Wrist positions
        public static double wristVertical = 0.545;
        public static double wristHorizontal = 0.175;

        // Hand positions
        public static double handOpen = 0.5;
        public static double handClosed = 0.775;
        public static double handHalfOpen = 0.7;
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
        public void transferTwo() {
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

    public void actOne() {
        try {
            robot.depositSlide.transfer();
            Robot.HardwareDevices.depositSlide.setPower(DepositSlide.DepositSlidePower.move/2);
            robot.outtake.wrist.horizontal();
            robot.intake.elbow.up();
            robot.intake.arm.transfer();
            robot.intake.wrist.horizontal();
            Thread.sleep(400);
            Robot.HardwareDevices.depositSlide.setPower(DepositSlide.DepositSlidePower.move);
            robot.intake.hand.halfOpen();
            Thread.sleep(400);
            robot.intake.hand.close();
            robot.outtake.arm.forward();
            robot.outtake.hand.open();
            robot.intakeSlide.retract();
            intakeSlideL.setPower(IntakeSlide.IntakeSlidePower.move/2);
            intakeSlideR.setPower(IntakeSlide.IntakeSlidePower.move/2);
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
