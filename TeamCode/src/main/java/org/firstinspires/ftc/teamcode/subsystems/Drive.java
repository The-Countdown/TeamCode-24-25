package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Trajectory;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.limelightvision.LLResult;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.main.Auto.RoadRunner.MecanumDrive;

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

    public void move(double forwardAmount, double strafeAmount) {
        robot.roadRunner.updatePoseEstimate();
        Pose2d currentPose = robot.roadRunner.pose;
        MecanumDrive.PARAMS.timeout = 0;

        Pose2d targetPose = new Pose2d(
                currentPose.position.x + forwardAmount,
                currentPose.position.y + strafeAmount,
                currentPose.heading.real
        );

        TrajectoryActionBuilder trajectory = robot.roadRunner.actionBuilder(currentPose)
                .strafeTo(new Vector2d(targetPose.position.x, targetPose.position.y));

        trajectory.build();
    }
}