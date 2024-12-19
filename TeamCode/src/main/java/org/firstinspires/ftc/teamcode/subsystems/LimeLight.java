package org.firstinspires.ftc.teamcode.subsystems;

import android.util.JsonReader;

import com.google.gson.Gson;
import com.qualcomm.hardware.limelightvision.LLResult;

import java.io.BufferedReader;
import java.io.InputStreamReader;
import java.net.HttpURLConnection;
import java.net.URL;

public class LimeLight extends Robot.HardwareDevices {
    private Robot robot;
    private final String urlString = "http://172.29.0.1:5807/results";

    public LimeLight(Robot robot) {
        this.robot = robot;
    }

    public void lineUp() {
        LLResult result;
        result = limelight.getLatestResult();

        if (result == null) {
            robot.opMode.telemetry.addData("Limelight", "No Targets");
            robot.opMode.telemetry.update();
            return;
        }

        double tx = result.getTx();
        double ty = result.getTy();

        robot.opMode.telemetry.addData("tx", tx);
        robot.opMode.telemetry.addData("ty", ty);

        Robot.HardwareDevices.intakeClawAngle.setPosition(Intake.IntakePosition.wristHorizontal - (tx / Robot.servoToDegrees));
        robot.opMode.telemetry.addData("angle", Intake.IntakePosition.wristHorizontal - (tx / Robot.servoToDegrees));

        robot.opMode.telemetry.update();
    }

    public void goToLimelightPos(double targetTx, double targetTy, double error) {
        LLResult result = limelight.getLatestResult();

        double currentTx = result.getTx();
        double currentTy = result.getTy();

        while ((Math.abs(targetTx) - Math.abs(currentTx) > error) && (Math.abs(targetTy) - Math.abs(currentTy) > error)) {
            if (currentTx <= targetTx) {
                //Strafe right
            }

            if (currentTx > targetTx) {
                //Strafe left
            }

            if (currentTy <= targetTy) {
                //Strafe forward
            }

            if (currentTy > targetTy) {
                //Strafe backward
            }
        }
    }

    public String fetchLimelightData() throws Exception {
        URL url = new URL(urlString);
        HttpURLConnection connection = (HttpURLConnection) url.openConnection();
        connection.setRequestMethod("GET");
        connection.setRequestProperty("Content-Type", "application/json");
        connection.setConnectTimeout(5000);
        connection.setReadTimeout(5000);

        int status = connection.getResponseCode();
        if (status != 200) {
            throw new Exception("Failed to fetch data from Limelight");
        }

        BufferedReader in = new BufferedReader(new InputStreamReader(connection.getInputStream()));
        String inputLine;

        StringBuilder content = new StringBuilder();
        while ((inputLine = in.readLine()) != null) {
            content.append(inputLine);
        }
        in.close();

        connection.disconnect();

        return content.toString();
    }

    public double getBlockOrientation() {
        String limeLightData = "";
        try {
            limeLightData = fetchLimelightData();
        } catch (Exception e) {
            robot.opMode.telemetry.addData("Error", e.getMessage());
            robot.opMode.telemetry.update();
            return 0;
        }

        if (limeLightData.equals("")) {
            return 0;
        }

        // Convert limelightData JSON to object
        LimeLightResults results = null;
        try {
            Gson gson = new Gson();
            results = gson.fromJson(limeLightData, LimeLightResults.class);
        } catch (Exception e) {
            robot.opMode.telemetry.addData("Error", e.getMessage());
            robot.opMode.telemetry.update();
            return 0;
        }


        if (results.Retro[0].pts.length > 5) {
            return 0;
        }

        double[][] pts = results.Retro[0].pts;
        double maxDistance = 0;
        double[] point1 = new double[2];
        double[] point2 = new double[2];

        for (int i = 0; i < pts.length; i++) {
            for (int j = i + 1; j < pts.length; j++) {
                double distance = Math.sqrt(Math.pow(pts[j][0] - pts[i][0], 2) + Math.pow(pts[j][1] - pts[i][1], 2));
                if (distance > maxDistance) {
                    maxDistance = distance;
                    point1 = pts[i];
                    point2 = pts[j];
                }
            }
        }

        double x1 = point1[0];
        double y1 = point1[1];
        double x2 = point2[0];
        double y2 = point2[1];

        // Calculate angle
        double angle = Math.atan2(y2 - y1, x2 - x1);
        return Math.toDegrees(angle);
    }

    private class LimeLightResults {
        public PtsRoot[] Retro;
        public class PtsRoot {
            public double[][] pts;
        }
    }
}



