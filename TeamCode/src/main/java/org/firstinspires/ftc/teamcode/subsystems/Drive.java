package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.main.Auto.RoadRunner.MecanumDrive;

public class Drive extends Robot.HardwareDevices {

    public MecanumDrive drive;

    public Drive(HardwareMap hardwareMap) {
        Pose2d initialPose = getRobotPosLimeLight();
        if (initialPose == null) {
            initialPose = new Pose2d(0, 0, 0);
        }
        this.drive = new MecanumDrive(hardwareMap, initialPose);
    }

    public Pose2d getRobotPosLimeLight() {
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

    public Pose2d getRobotPosRoadRunner() {
        return drive.pose;
    }

    public Pose2d getRobotPos() {
        Pose2d limelightPose = getRobotPosLimeLight();
        Pose2d roadRunnerPose = getRobotPosRoadRunner();

        if (limelightPose == null || roadRunnerPose == null) {
            return null;
        }

        double avgX = (limelightPose.position.x + roadRunnerPose.position.x) / 2;
        double avgY = (limelightPose.position.y + roadRunnerPose.position.y) / 2;
        double avgHeading = (limelightPose.heading.real + roadRunnerPose.heading.real) / 2;

        return new Pose2d(avgX, avgY, avgHeading);
    }

    public class Blue {
        public void basket() {
            Pose2d currentPose = getRobotPos();
            {
                Actions.run(
                        drive.actionBuilder(currentPose)
                                .splineToSplineHeading(new Pose2d(57.88, 56.86, Math.toRadians(45)), Math.toRadians(45))
                                .build());
            }
        }
        public void basketBlock() {
            Pose2d currentPose = getRobotPos();
            {
                Actions.runBlocking(
                        drive.actionBuilder(currentPose)
                                .splineToSplineHeading(new Pose2d(57.88, 56.86, Math.toRadians(45)), Math.toRadians(45))
                                .build());
            }

        }
        public void specimen() {
            Pose2d currentPose = getRobotPos();
            {
                Actions.run(
                        drive.actionBuilder(currentPose)
                                .splineToSplineHeading(new Pose2d(0, 35, Math.toRadians(90)), Math.toRadians(90))
                                .build());
            }
        }
        public void specimenBlocking() {
            Pose2d currentPose = getRobotPos();
            {
                Actions.runBlocking(
                        drive.actionBuilder(currentPose)
                                .splineToSplineHeading(new Pose2d(0, 35, Math.toRadians(90)), Math.toRadians(90))
                                .build());
            }
        }
    }
    public class Red {
        public void basket() {
            Pose2d currentPose = getRobotPos();
            {
                Actions.run(
                        drive.actionBuilder(currentPose)
                                .splineToSplineHeading(new Pose2d(-57.88, -56.86, Math.toRadians(45)), Math.toRadians(45)) // Flipped signs
                                .build());
            }
        }
        public void basketBlock() {
            Pose2d currentPose = getRobotPos();
            {
                Actions.runBlocking(
                        drive.actionBuilder(currentPose)
                                .splineToSplineHeading(new Pose2d(-57.88, -56.86, Math.toRadians(45)), Math.toRadians(45)) // Flipped signs
                                .build());
            }
        }
        public void specimen() {
            Pose2d currentPose = getRobotPos();
            {
                Actions.run(
                        drive.actionBuilder(currentPose)
                                .splineToSplineHeading(new Pose2d(0, -35, Math.toRadians(90)), Math.toRadians(90)) // Flipped sign on Y
                                .build());
            }
        }
        public void specimenBlocking() {
            Pose2d currentPose = getRobotPos();
            {
                Actions.runBlocking(
                        drive.actionBuilder(currentPose)
                                .splineToSplineHeading(new Pose2d(0, -35, Math.toRadians(90)), Math.toRadians(90)) // Flipped sign on Y
                                .build());
            }
        }
    }
    public Blue blue = new Blue();
    public Red red = new Red();
}