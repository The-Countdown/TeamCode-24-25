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

    public LLResult getLimeLightResult() {
        return limelight.getLatestResult();
    }

    public boolean goToLimelightPos(double targetTx, double targetTy, double error) {
        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {
            double currentTx = result.getTx();
            double currentTy = result.getTy();
            robot.telemetry.addData("tx: ", currentTx);
            robot.telemetry.addData("ty: ", currentTx);

            double errorx = Math.abs(currentTx) - Math.abs(targetTx);
            boolean boolx = errorx > error;
            double errory = Math.abs(currentTy) - Math.abs(targetTy);
            boolean booly = errory > error;
            robot.telemetry.addData("errorx: ", errorx);
            robot.telemetry.addData("boolx: ", boolx);
            robot.telemetry.addData("errory: ", errory);
            robot.telemetry.addData("booly: ", booly);

            if (boolx || booly) {
                robot.telemetry.addData("yes: ", true);
                if (currentTx >= targetTx) {
                    robot.drive.movePower(0, 0.6, 0);
                    robot.telemetry.addData("left: ", true);
                }

                if (currentTx < targetTx) {
                    robot.drive.movePower(0, -0.6, 0);
                    robot.telemetry.addData("right: ", true);
                }

                if (currentTy <= targetTy) {
                    robot.drive.movePower(0.5, 0, 0);
                    robot.telemetry.addData("forwards: ", true);
                }

                if (currentTy > targetTy) {
                    robot.drive.movePower(-0.5, 0, 0);
                    robot.telemetry.addData("backwards: ", true);
                }
            } else {
                robot.drive.movePower(0, 0, 0);
                return true;
            }
        } else {
            robot.drive.movePower(0, 0, 0);
            return false;
        }
        robot.telemetry.update();
        return false;
    }

    public void goToLimeLightPosLoop (double targetTx, double targetTy, double error) {
        while(!goToLimelightPos(targetTx, targetTy, error));
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

    public void limeLightInit(int pipeline, int pollRate) {
        Robot.HardwareDevices.limelight.setPollRateHz(pollRate);
        Robot.HardwareDevices.limelight.start();
        Robot.HardwareDevices.limelight.pipelineSwitch(pipeline);
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

        if (results.Retro.length == 0) {
            return 0;
        }

        if (results.Retro[0].pts.length > 5) {
            return 0;
        }

        double[][] pts = results.Retro[0].pts;

        // find a rectangle of the maximum area
        double maxArea = 0;
        int maxIndex = 0;

        for (int i = 0; i < pts.length; i++) {
            double[] pt1 = pts[i];
            double[] pt2 = pts[(i + 1) % pts.length];
            double[] pt3 = pts[(i + 2) % pts.length];

            double area = Math.abs((pt1[0] * (pt2[1] - pt3[1]) + pt2[0] * (pt3[1] - pt1[1]) + pt3[0] * (pt1[1] - pt2[1])) / 2);
            if (area > maxArea) {
                maxArea = area;
                maxIndex = i;
            }
        }

        double[] pt1 = pts[maxIndex];
        double[] pt2 = pts[(maxIndex + 1) % pts.length];
        double[] pt3 = pts[(maxIndex + 2) % pts.length];

        double[] side1 = {pt2[0] - pt1[0], pt2[1] - pt1[1]};
        double[] side2 = {pt3[0] - pt2[0], pt3[1] - pt2[1]};

        // Ensure side1 is the longer side
        if (Math.sqrt(side1[0] * side1[0] + side1[1] * side1[1]) < Math.sqrt(side2[0] * side2[0] + side2[1] * side2[1])) {
            double[] temp = side1;
            side1 = side2;
            side2 = temp;
        }

        // Calculate the angle of the longer side relative to the x-axis (camera)
        double angle = Math.atan2(side1[1], side1[0]);

        robot.opMode.telemetry.addData("Angle", Math.toDegrees(angle));
        robot.opMode.telemetry.addData("Area", maxArea);
        robot.opMode.telemetry.addData("Side 1 Length", Math.sqrt(side1[0] * side1[0] + side1[1] * side1[1]));
        robot.opMode.telemetry.addData("Side 2 Length", Math.sqrt(side2[0] * side2[0] + side2[1] * side2[1]));
        robot.opMode.telemetry.addData("Servo", Robot.HardwareDevices.intakeClawAngle.getPosition());

        double degrees = Math.toDegrees(angle);

        if (degrees < 90 && degrees > -90) {
            return 180 - Math.abs(degrees);
        }
        return degrees;
    }

    private class LimeLightResults {
        public PtsRoot[] Retro;
        public class PtsRoot {
            public double[][] pts;
        }
    }
}



