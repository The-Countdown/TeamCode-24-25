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
        public static double armUp = 0.66;
        public static double armDown = 0.59;
        public static double armRest = 0.09;
        public static double armTransfer = 0.91;

        // Elbow positions
        public static double elbowUp = 0.43;
        public static double elbowRest = 0.39;
        public static double elbowDown = 0.05;
        public static double elbowTransfer = 0.75;

        // Wrist positions
        public static double wristVertical = 0.66;
        public static double wristHorizontal = 0.94;
        public static double wristHorizontalFlip = 0.38;
        public static double wristAutoRight = 0.83;

        // Hand positions
        public static double handOpen = 0.6;
        public static double handClosed = 0.83;
        public static double handHalfOpen = 0.76;
    }

    public class Arm {
        public void up() {
            intakePitchL.setPosition(IntakePosition.armUp);
            intakePitchR.setPosition(IntakePosition.armUp);
        }
        public void down() {
            intakePitchL.setPosition(IntakePosition.armDown);
            intakePitchR.setPosition(IntakePosition.armDown);
        }
        public void rest() {
            intakePitchL.setPosition(IntakePosition.armRest);
            intakePitchR.setPosition(IntakePosition.armRest);
        }
        public void transfer() {
            intakePitchL.setPosition(IntakePosition.armTransfer);
            intakePitchR.setPosition(IntakePosition.armTransfer);
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
        public void horizontalFlip() {
            intakeClawAngle.setPosition(IntakePosition.wristHorizontalFlip);
        }
        public void autoRight() {
            intakeClawAngle.setPosition(IntakePosition.wristAutoRight);
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
            robot.depositSlide.transferUp();
            Robot.HardwareDevices.depositSlide.setPower(DepositSlide.DepositSlidePower.move/2);
            robot.intake.elbow.up();
            robot.intake.arm.transfer();
            robot.intake.wrist.horizontal();
            Thread.sleep(200);
            Robot.HardwareDevices.depositSlide.setPower(DepositSlide.DepositSlidePower.move);
            robot.intake.hand.halfOpen();
            robot.outtake.wrist.horizontal();
            Thread.sleep(400);
            robot.intake.hand.close();
            robot.intake.wrist.horizontalFlip();
            robot.outtake.hand.open();
            robot.outtake.arm.transfer();
            robot.intakeSlide.retract();
            intakeSlideL.setPower(IntakeSlide.IntakeSlidePower.move/1.5);
            intakeSlideR.setPower(IntakeSlide.IntakeSlidePower.move/1.5);
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
