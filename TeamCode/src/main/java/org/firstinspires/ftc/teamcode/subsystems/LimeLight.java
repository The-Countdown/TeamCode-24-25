package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.limelightvision.LLResult;

import java.io.BufferedReader;
import java.io.InputStreamReader;
import java.net.HttpURLConnection;
import java.net.URL;

public class LimeLight extends Robot.HardwareDevices {
    private Robot robot;
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
    public String fetchLimelightData(String urlString) throws Exception {
        URL url = new URL(urlString);
        HttpURLConnection connection = (HttpURLConnection) url.openConnection();
        connection.setRequestMethod("GET");

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
}



