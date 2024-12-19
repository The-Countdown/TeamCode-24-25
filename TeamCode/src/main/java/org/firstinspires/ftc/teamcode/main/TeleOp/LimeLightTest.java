package org.firstinspires.ftc.teamcode.main.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.json.JSONArray;
import org.json.JSONObject;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "LimeLight")
@Config
public class LimeLightTest extends LinearOpMode {

    @Override
    public void runOpMode() {
        Robot robot = new Robot(this);

        FtcDashboard dashboard = FtcDashboard.getInstance();
        TelemetryPacket packet = new TelemetryPacket();

        waitForStart();

        LimeLightThread limeLightThread = new LimeLightThread(this, robot);
        Thread limeLight = new Thread(limeLightThread);
        limeLight.start();

        while (opModeIsActive()) {
            LLResult result;
            result = Robot.HardwareDevices.limelight.getLatestResult();
            try {
                String jsonResponse = robot.limeLight.fetchLimelightData("http://172.28.0.1:5807/results");

                JSONObject jsonObject = new JSONObject(jsonResponse);
                JSONArray cornersArray = jsonObject.getJSONArray("tcornxy");

                if (cornersArray.length() >= 8) {
                    telemetry.addLine("Target Corners:");
                    for (int i = 0; i < cornersArray.length(); i += 2) {
                        double x = cornersArray.getDouble(i);
                        double y = cornersArray.getDouble(i + 1);
                        telemetry.addData("Corner " + (i / 2), "(%.2f, %.2f)", x, y);
                        packet.put("Corner" + (i / 2), "(%.2f, %.2f)" + x + y);
                    }
                } else {
                    telemetry.addLine("No corners detected.");
                    packet.addLine("No corners detected.");
                }
            } catch (Exception e) {
                telemetry.addLine("Error: " + e.getMessage());
                packet.addLine("Error:" + e.getMessage());
            }
            telemetry.addData("tx", result.getTx());
            telemetry.addData("ty", result.getTy());
            telemetry.update();
            dashboard.sendTelemetryPacket(packet);
        }
        Robot.hasResetEncoders = false;
    }
}