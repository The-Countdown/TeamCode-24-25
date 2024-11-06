package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.subsystems.Robot.rb;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.limelightvision.LLResult;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

public class Drive extends Robot.HardwareDevices {

    public void move(double stickX, double stickY) {

    }

    public void moveField(double stickX, double stickY) {

    }

    public Pose2d getLimeLightPos() {
        Robot.HardwareDevices.limelight.setPollRateHz(100);
        Robot.HardwareDevices.limelight.start();
        Robot.HardwareDevices.limelight.pipelineSwitch(3);
        LLResult result = Robot.HardwareDevices.limelight.getLatestResult();

        double robotYaw = Robot.HardwareDevices.imu.getRobotYawPitchRollAngles().getYaw();
        Robot.HardwareDevices.limelight.updateRobotOrientation(robotYaw);

        Pose3D botPose_mt2 = result.getBotpose_MT2();

        double xLimeLight = botPose_mt2.getPosition().x * 39.3700787402; // Convert from meters to inches
        double yLimeLight = botPose_mt2.getPosition().y * 39.3700787402; // Convert from meters to inches
        double headingLimeLight = botPose_mt2.getOrientation().getYaw();

        return new Pose2d(xLimeLight, yLimeLight, headingLimeLight);
    }

    public void toPose (double x, double y, double angle) {
        rb.updatePose();
        Actions.runBlocking(rb.dreadDrive.actionBuilder(rb.dreadDrive.pose)
                .splineToSplineHeading(new Pose2d(x, y, Math.toRadians(angle)), Math.toRadians(0))
                .build());
    }
}