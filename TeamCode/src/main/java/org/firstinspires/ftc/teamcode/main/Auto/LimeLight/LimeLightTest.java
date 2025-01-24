package org.firstinspires.ftc.teamcode.main.Auto.LimeLight;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.subsystems.LimeLight;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

@Disabled
@Autonomous(name = "LimeLightTest")
public class LimeLightTest extends LinearOpMode {

    @Config
    public static class LimeLightVariables {
        public static double xMulti = 4;
        public static double xAdd = 3.5;
        public static double yMulti = 4;
        public static double height = 5.8; // height of the camera in inches
    }

    @Override
    public void runOpMode() {
        Robot robot = new Robot(this, new Pose2d(0, 57.11, Math.toRadians(0)));
        robot.limeLight.limeLightInit(LimeLight.Pipelines.Red,30);
        robot.intake.arm.up();

        waitForStart();

        while (opModeIsActive()) {
            LLResult result = robot.limeLight.getLimeLightResult();
            double currentTx = result.getTx();
            double currentTy = result.getTy();
            double errorx = currentTx - 0.1;
            double moveAmountX = errorx;
            double errory = currentTy - 0.1;
            double moveAmountY = errory;
            double xDistance = LimeLightVariables.height * Math.tan(Math.toRadians(errory));
            double yDistance = LimeLightVariables.height * Math.tan(Math.toRadians(errorx));
            telemetry.addData("xValue", String.valueOf((xDistance * LimeLightVariables.xMulti) + LimeLightVariables.xAdd));
            telemetry.addData("yValue", String.valueOf(-(yDistance * LimeLightVariables.yMulti)));
            telemetry.addLine();
            telemetry.addData("Flashlight Distance", Robot.HardwareDevices.flashLight.getDistance(DistanceUnit.CM));
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
        Robot.HardwareDevices.limelight.stop();
    }
}
