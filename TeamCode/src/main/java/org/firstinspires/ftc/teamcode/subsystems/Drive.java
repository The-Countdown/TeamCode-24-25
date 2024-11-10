package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.limelightvision.LLResult;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

public class Drive extends Robot.HardwareDevices {
    private Robot robot;

    public Drive(Robot robot) {
        this.robot = robot;
    }

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

    public void toPose (Pose2d pose, Double tangent) {
        if (tangent == null) {
            tangent = 0d;
        }
        robot.updatePose();
        Actions.runBlocking(robot.dreadDrive.actionBuilder(robot.dreadDrive.pose)
                .splineToLinearHeading(pose, tangent)
                .build());
    }

    public void toPose (Pose2d pose) {toPose(pose, null);}
}