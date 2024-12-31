package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
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

    public void moveAmount(double x, double y, double angle) {
        robot.roadRunner.updatePoseEstimate();
        Pose2d currentPose = robot.roadRunner.pose;
        MecanumDrive.PARAMS.timeout = 0;

        double outX = currentPose.position.x + x;
        double outY = currentPose.position.y + y;
        double outAngle = currentPose.heading.real + angle;

//        Pose2d targetPose = new Pose2d(
//                currentPose.position.x + x,
//                currentPose.position.y + y,
//                currentPose.heading.real + angle);
        Vector2d targetPose = new Vector2d(
                currentPose.position.x + x,
                currentPose.position.y + y);

//        TrajectoryActionBuilder trajectory = robot.roadRunner.actionBuilder(currentPose)
//                .splineToLinearHeading(targetPose, outAngle); //TODO: Change depending on usage
        TrajectoryActionBuilder trajectory = robot.roadRunner.actionBuilder(currentPose)
                .strafeTo(targetPose); //TODO: Change depending on usage

        trajectory.build();

        MecanumDrive.PARAMS.timeout = 2;
    }
    public void moveTo(double x, double y, double angle) {
        robot.roadRunner.updatePoseEstimate();
        Pose2d currentPose = robot.roadRunner.pose;
        MecanumDrive.PARAMS.timeout = 0;

        TrajectoryActionBuilder trajectory = robot.roadRunner.actionBuilder(currentPose)
                .splineToLinearHeading(new Pose2d(x, y, angle), angle); //TODO: Change depending on usage

        trajectory.build();

        MecanumDrive.PARAMS.timeout = 2;
    }
    public void movePower(double forward, double strafe, double rotation) {
        robot.roadRunner.setDrivePowers(new PoseVelocity2d(
                new Vector2d(
                        -forward,
                        -strafe
                ),
                -rotation
        ));
    }
}