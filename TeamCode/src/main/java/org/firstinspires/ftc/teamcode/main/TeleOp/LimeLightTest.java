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
        robot.limeLight.limeLightInit(0,100);

        waitForStart();

        LimeLightThread limeLightThread = new LimeLightThread(this, robot);
        Thread limeLight = new Thread(limeLightThread);
        limeLight.start();

        while (opModeIsActive()) {
            if (isStopRequested()) break;
        }
        Robot.hasResetEncoders = false;
    }
}