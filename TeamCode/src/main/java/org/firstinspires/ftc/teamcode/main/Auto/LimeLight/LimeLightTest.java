package org.firstinspires.ftc.teamcode.main.Auto.LimeLight;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "LimeLightTest")
public class LimeLightTest extends LinearOpMode {
    Limelight3A limelight;

    @Override
    public void runOpMode() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.start();
        limelight.pipelineSwitch(3);

        waitForStart();

        while (opModeIsActive()) {
            LLResult result = limelight.getLatestResult();
            if (result != null && result.isValid()) {
                double tx = result.getTx(); // How far left or right the target is (degrees)
                double ty = result.getTy(); // How far up or down the target is (degrees)
                double ta = result.getTa(); // How big the target looks (0%-100% of the image)
                int aprilTagID = result.getFiducialResults().get(0).getFiducialId();
                double robotx = result.getFiducialResults().get(0).getRobotPoseFieldSpace().getPosition().x;
                double roboty = result.getFiducialResults().get(0).getRobotPoseFieldSpace().getPosition().y;

                telemetry.addData("Target X", tx);
                telemetry.addData("Target Y", ty);
                telemetry.addData("Target Area", ta);
                telemetry.addData("April id", aprilTagID);
                telemetry.addData("robotX", robotx);
                telemetry.addData("robotY", roboty);
                telemetry.update();

            } else {
                telemetry.addData("Limelight", "No Targets");
                telemetry.update();
            }
        }
    }
}
