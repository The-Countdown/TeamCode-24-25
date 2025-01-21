package org.firstinspires.ftc.teamcode.subsystems;

import android.util.JsonReader;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.google.gson.Gson;
import com.qualcomm.hardware.limelightvision.LLResult;

import org.firstinspires.ftc.teamcode.subsystems.actions.outtake.OuttakeHighNet;
import org.firstinspires.ftc.teamcode.subsystems.actions.outtake.OuttakeTransferPrep;

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

    public LLResult getLimeLightResult() {
        return limelight.getLatestResult();
    }

    public TrajectoryActionBuilder goToLimelightPos(double targetTx, double targetTy, double error) {
        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {
            double currentTx = result.getTx();
            double currentTy = result.getTy();

            double height = 6.3; // height of the camera in inches
            double errorx = currentTx - targetTx;
            double moveAmountX = errorx;
            double errory = currentTy - targetTy;
            double moveAmountY = errory;
            double xDistance = height * Math.tan(Math.toRadians(errory));
            double yDistance = height * Math.tan(Math.toRadians(errorx));
            robot.telemetry.addData("Moving to x:", moveAmountX);
            robot.telemetry.addData("Moving to y:", moveAmountY);
            return robot.drive.moveAmount(0 ,-(yDistance * 1.4), 0);
        } else {
            return null;
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

    public void limeLightInit(int pipeline, int pollRate) {
        Robot.HardwareDevices.limelight.setPollRateHz(pollRate);
        Robot.HardwareDevices.limelight.start();
        Robot.HardwareDevices.limelight.pipelineSwitch(pipeline);  // Yellow = 0, Blue = 1, Red = 2, April = 3
    }

    public double getBlockOrientation() {
        String limeLightData = "";
        try {
            limeLightData = fetchLimelightData();
        } catch (Exception e) {
            robot.opMode.telemetry.addData("Error", e.getMessage());
            return 0;
        }

        if (limeLightData.equals("")) {
            robot.opMode.telemetry.addData("Error", "No data received from Limelight");
            return 0;
        }

        // Convert limelightData JSON to object
        LimeLightResults results = null;
        try {
            Gson gson = new Gson();
            results = gson.fromJson(limeLightData, LimeLightResults.class);
        } catch (Exception e) {
            robot.opMode.telemetry.addData("Error", e.getMessage());
            return 0;
        }

        if (results.Retro.length == 0) {
            robot.opMode.telemetry.addData("Error", "No Retro data received from Limelight");
            return 0;
        }

        double[][] pts = results.Retro[0].pts;

        // find a rectangle of the maximum area with the long side approximately 2.57 times the short side
        int maxIndex = 0;
        double bestRatioDiff = Double.MAX_VALUE;

        for (int i = 0; i < pts.length; i++) {
            double[] pt1 = pts[i];
            double[] pt2 = pts[(i + 1) % pts.length];
            double[] pt3 = pts[(i + 2) % pts.length];

            double[] side1 = {pt2[0] - pt1[0], pt2[1] - pt1[1]};
            double[] side2 = {pt3[0] - pt2[0], pt3[1] - pt2[1]};

            double length1 = Math.sqrt(side1[0] * side1[0] + side1[1] * side1[1]);
            double length2 = Math.sqrt(side2[0] * side2[0] + side2[1] * side2[1]);

            double longSide = Math.max(length1, length2);
            double shortSide = Math.min(length1, length2);
            double ratio = longSide / shortSide;
            double ratioDiff = Math.abs(ratio - 2.57);

            if (ratioDiff < bestRatioDiff) {
                maxIndex = i;
                bestRatioDiff = ratioDiff;
            }
        }

        double[] pt1 = pts[maxIndex];
        double[] pt2 = pts[(maxIndex + 1) % pts.length];
        double[] pt3 = pts[(maxIndex + 2) % pts.length];

        double[] side1 = {pt2[0] - pt1[0], pt2[1] - pt1[1]};
        double[] side2 = {pt3[0] - pt2[0], pt3[1] - pt2[1]};

        double side1Length = Math.sqrt(side1[0] * side1[0] + side1[1] * side1[1]);
        double side2Length = Math.sqrt(side2[0] * side2[0] + side2[1] * side2[1]);

        boolean isSide1OutOfFrame = false;
        boolean isSide2OutOfFrame = false;

//        //if side1[0] or side2[0] is less than 1 or above 638 make it so that the side length is the other side but with the ratio
//        if (side1[0] < 1 || side1[0] > 638 || side1[1] < 1 || side1[1] > 478) {
//            isSide1OutOfFrame = true;
//        }
//        if (side2[0] < 1 || side2[0] > 638 || side2[1] < 1 || side2[1] > 478) {
//            isSide2OutOfFrame = true;
//        }
//
//        if (isSide1OutOfFrame && isSide2OutOfFrame) {
//            robot.opMode.telemetry.addData("Error", "Both sides are out of frame");
//            return 0;
//        }
//
//        if (isSide1OutOfFrame) {
//            side1Length = side2Length * 2.57;
//        } else if (isSide2OutOfFrame) {
//            side2Length = side1Length * 2.57;
//        }

        // Ensure side1 is the longer side
        if (side1Length < side2Length) {
            side1 = side2;
        }

        // Calculate the angle of the longer side relative to the x-axis (camera)
        double angle = Math.atan2(side1[1], side1[0]);

        angle = Math.toDegrees(angle);

        if (angle < 0)
        {
            angle += 180;
        }

        if (angle > 90)
        {
            angle = 180 - angle;
            angle *= -1;
        }

        angle += 90;

        angle = 180 - angle;

        robot.opMode.telemetry.addData("Raw Angle", angle);
        return angle;
    }

    private class LimeLightResults {
        public PtsRoot[] Retro;
        public class PtsRoot {
            public double[][] pts;
        }
    }
}