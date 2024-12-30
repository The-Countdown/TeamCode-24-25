package org.firstinspires.ftc.teamcode.main.Auto.LimeLight;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystems.Robot;

@Autonomous(name = "LimeLightTest")
public class LimeLightTest extends LinearOpMode {

    @Override
    public void runOpMode() {
        Robot robot = new Robot(this, new Pose2d(0, 57.11, Math.toRadians(0)));
        robot.limeLight.limeLightInit(0,100);
        robot.intake.arm.up();

        waitForStart();

        while (opModeIsActive()) {
            LLResult result = robot.limeLight.getLimeLightResult();
            if (result != null && result.isValid()) {
                double tx = result.getTx(); // How far left or right the target is (degrees)
                double ty = result.getTy(); // How far up or down the target is (degrees)
                double ta = result.getTa(); // How big the target looks (0%-100% of the image)
                telemetry.addData("Target X", tx);
                telemetry.addData("Target Y", ty);
                telemetry.addData("Target Area", ta);
                if (!result.getFiducialResults().isEmpty()) {
                    int aprilTagID = result.getFiducialResults().get(0).getFiducialId();
                    double robotx = result.getFiducialResults().get(0).getRobotPoseFieldSpace().getPosition().x;
                    double roboty = result.getFiducialResults().get(0).getRobotPoseFieldSpace().getPosition().y;
                    telemetry.addData("April id", aprilTagID);
                    telemetry.addData("robotX", robotx);
                    telemetry.addData("robotY", roboty);
                }
                telemetry.update();
            } else {
                telemetry.addData("Limelight", "No Targets");
                telemetry.update();
            }
        }
    }
}
