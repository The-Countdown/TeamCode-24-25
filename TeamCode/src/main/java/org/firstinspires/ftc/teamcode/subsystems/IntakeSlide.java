package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class IntakeSlide extends Robot.HardwareDevices {
    private Robot robot;

    public IntakeSlide(Robot robot) {
        this.robot = robot;
    }

    @Config
    public static class IntakeSlidePosition {
        public static int retracted = 0;
        public static int handOff = 1150;
        public static int minimum = 0;
        public static int maximum = 1500;
        public static int tolerance = 5;
        public static int stepRange = 100;

        public static boolean magRetracting = false;
    }
    @Config
    public static class IntakeSlidePower {
        public static double stop = 0;
        public static double move = 1;
    }
    public void stop() {
        intakeSlideL.setPower(IntakeSlidePower.stop);
        intakeSlideR.setPower(IntakeSlidePower.stop);
    }

    public void moveTo(int amount){
        if (amount < IntakeSlidePosition.minimum) {
            amount = IntakeSlidePosition.minimum;
        } else if (amount > IntakeSlidePosition.maximum) {
            amount = IntakeSlidePosition.maximum;
        }

        intakeSlideL.setTargetPositionTolerance(IntakeSlidePosition.tolerance);
        intakeSlideR.setTargetPositionTolerance(IntakeSlidePosition.tolerance);
        intakeSlideL.setTargetPosition(amount);
        intakeSlideR.setTargetPosition(amount);
        intakeSlideL.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        intakeSlideR.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        intakeSlideL.setPower(IntakeSlidePower.move);
        intakeSlideR.setPower(IntakeSlidePower.move);
    }

    public int avg() {
        return (intakeSlideL.getCurrentPosition() + intakeSlideR.getCurrentPosition()) / 2;
    }

    public void magRetract() {
        IntakeSlidePosition.magRetracting = true;
        intakeSlideL.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        intakeSlideR.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        intakeSlideL.setPower(-IntakeSlide.IntakeSlidePower.move);
        intakeSlideR.setPower(-IntakeSlide.IntakeSlidePower.move);
        while (!Robot.HardwareDevices.intakeMagnetL.isPressed() && !Robot.HardwareDevices.intakeMagnetR.isPressed());
        intakeSlideL.setPower(IntakeSlide.IntakeSlidePower.stop);
        intakeSlideR.setPower(IntakeSlide.IntakeSlidePower.stop);
        intakeSlideL.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        intakeSlideR.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        intakeSlideL.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        intakeSlideR.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        intakeSlideL.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        intakeSlideR.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        intakeSlideL.setTargetPosition(avg());
        intakeSlideR.setTargetPosition(avg());
        IntakeSlidePosition.magRetracting = false;
    }

    public void retract() {
        intakeSlideL.setTargetPositionTolerance(IntakeSlidePosition.tolerance);
        intakeSlideR.setTargetPositionTolerance(IntakeSlidePosition.tolerance);
        intakeSlideL.setTargetPosition(IntakeSlidePosition.retracted);
        intakeSlideR.setTargetPosition(IntakeSlidePosition.retracted);
        intakeSlideL.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        intakeSlideR.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        intakeSlideL.setPower(IntakeSlidePower.move);
        intakeSlideR.setPower(IntakeSlidePower.move);
    }
    public void handOff() {
        intakeSlideL.setTargetPositionTolerance(IntakeSlidePosition.tolerance);
        intakeSlideR.setTargetPositionTolerance(IntakeSlidePosition.tolerance);
        intakeSlideL.setTargetPosition(IntakeSlidePosition.handOff);
        intakeSlideR.setTargetPosition(IntakeSlidePosition.handOff);
        intakeSlideL.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        intakeSlideR.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        intakeSlideL.setPower(IntakeSlidePower.move);
        intakeSlideR.setPower(IntakeSlidePower.move);
    }

    public void condensedMilk() {
        robot.intake.rest();
        magRetract();
    }
}