package org.firstinspires.ftc.teamcode.main.TeleOp;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Robot;


public class IntakeThread extends Robot.HardwareDevices implements Runnable {
    private final String inSequences;
    private final HardwareMap hardwareMap;
    private final Gamepad gamepad2;
    private final LinearOpMode opMode;
    private final Robot robot;

    public IntakeThread(String inSequences, HardwareMap hardwareMap, Gamepad gamepad2, LinearOpMode opMode, Robot robot) {
        this.inSequences = inSequences;
        this.hardwareMap = hardwareMap;
        this.gamepad2 = gamepad2;
        this.opMode = opMode;
        this.robot = robot;
    }

    @Override
    public void run() {
        while (opMode.opModeIsActive()) {
            try {
                int yStickLInt  = (int) (gamepad2.left_stick_y * 30);

                if (gamepad2.cross && (((intakeSlideL.getCurrentPosition() + intakeSlideR.getCurrentPosition()) / 2) > 75)) {
                    robot.intakeSlide.condense();
                } else if (gamepad2.cross && (((intakeSlideL.getCurrentPosition() + intakeSlideR.getCurrentPosition()) / 2) < 75)) {
                    robot.intakeSlide.pickUp();
                } else if (gamepad2.share) {
                    robot.intakeSlide.pickUpGround();
                } else if (gamepad2.dpad_down) {
                    robot.intakeSlide.condense();
                }

                if (gamepad2.left_stick_y != 0) {
                    if (((intakeSlideL.getTargetPosition() + intakeSlideR.getTargetPosition()) / 2) <= 1500) {
                        intakeSlideL.setTargetPosition(intakeSlideL.getTargetPosition() - yStickLInt);
                        intakeSlideR.setTargetPosition(intakeSlideR.getTargetPosition() - yStickLInt);
                        robot.intake.align();
                    }
                } else {
                    intakeSlideL.setTargetPosition(intakeSlideL.getTargetPosition());
                    intakeSlideR.setTargetPosition(intakeSlideR.getTargetPosition());
                    intakePitchL.setPosition(Intake.IntakePosition.downL - 0.005);
                    intakePitchR.setPosition(Intake.IntakePosition.downR + 0.005);
                }
            } catch (Exception e) {
                telemetry.addData("Error", e.getMessage());
                telemetry.update();
                throw new RuntimeException(e);
            }
        }
    }
}