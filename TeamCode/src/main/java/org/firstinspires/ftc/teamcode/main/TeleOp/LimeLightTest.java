package org.firstinspires.ftc.teamcode.main.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.subsystems.DepositSlide;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSlide;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "LimeLight")
@Config
public class LimeLightTest extends LinearOpMode {
    public static double yStickLMulti = 0.4;
    public static double xStickLMulti = 0.6;
    public static double xStickRMulti = 0.3;
    public static boolean depositMagnetPressed = false;

    @Override
    public void runOpMode() {
        Robot robot = new Robot(this);

        FtcDashboard dashboard = FtcDashboard.getInstance();

        waitForStart();

        LimeLightThread limeLightThread = new LimeLightThread(this, robot);
        Thread limeLight = new Thread(limeLightThread);
        limeLight.start();

        while (opModeIsActive()) {

        }
        Robot.hasResetEncoders = false;
    }
}
