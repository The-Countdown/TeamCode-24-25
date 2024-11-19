package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.limelightvision.LLResult;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

public class Drive extends Robot.HardwareDevices {
    private Robot robot;

    public Drive(Robot robot) {
        this.robot = robot;
    }

    public Pose2d getLimeLightPos() {
        Robot.HardwareDevices.limelight.setPollRateHz(100);
        Robot.HardwareDevices.limelight.start();
        Robot.HardwareDevices.limelight.pipelineSwitch(3);
        LLResult result = Robot.HardwareDevices.limelight.getLatestResult();

        double robotYaw = Robot.HardwareDevices.imu.getRobotYawPitchRollAngles().getYaw();
        Robot.HardwareDevices.limelight.updateRobotOrientation(robotYaw);

        Pose3D botPose_mt2 = result.getBotpose_MT2();

        double xLimeLight = botPose_mt2.getPosition().x * 39.3700787402;
        double yLimeLight = botPose_mt2.getPosition().y * 39.3700787402;
        double headingLimeLight = botPose_mt2.getOrientation().getYaw();

        return new Pose2d(xLimeLight, yLimeLight, headingLimeLight);
    }
}