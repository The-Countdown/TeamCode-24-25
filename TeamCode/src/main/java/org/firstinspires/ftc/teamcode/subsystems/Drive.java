package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.hardware.limelightvision.Limelight3A;


import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.main.Auto.LimeLight.LimeLightTest;
import org.firstinspires.ftc.teamcode.main.Auto.RoadRunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.main.Auto.RoadRunner.tuning.TuningOpModes;

public class Drive extends Robot.HardwareDevices {
    public double xCoordinate;
    public double yCoordinate;
    public double heading;

    public Pose2d getRobotPos() {
        Robot.HardwareDevices.limelight.setPollRateHz(100);
        Robot.HardwareDevices.limelight.start();
        Robot.HardwareDevices.limelight.pipelineSwitch(3);
        LLResult result = Robot.HardwareDevices.limelight.getLatestResult();

        double robotYaw = Robot.HardwareDevices.imu.getRobotYawPitchRollAngles().getYaw();
        Robot.HardwareDevices.limelight.updateRobotOrientation(robotYaw);

        if (result != null && result.isValid()) {
            Pose3D botpose_mt2 = result.getBotpose_MT2();
            if (botpose_mt2 != null) {
                double x = botpose_mt2.getPosition().x;
                double y = botpose_mt2.getPosition().y;
                double heading = botpose_mt2.getOrientation().getYaw();

                telemetry.addData("MT2 Location:", "(" + x + ", " + y + ")");
                telemetry.update();

                return new Pose2d(x, y, heading);
            }
        }
        return null;
    }
    public void setBeginPos(double xCoordinate, double yCoordinate, double heading) {
        this.xCoordinate = xCoordinate;
        this.yCoordinate = yCoordinate;
        this.heading = heading;
    }
    public void basket() {
        Pose2d beginPose = new Pose2d(xCoordinate, yCoordinate, heading);
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose); {
            Actions.runBlocking(
                    drive.actionBuilder(beginPose)
                            .splineToSplineHeading(new Pose2d(57.88, 56.86, Math.toRadians(45.00)), Math.toRadians(45.00))
                            .build());
        }
    }
}