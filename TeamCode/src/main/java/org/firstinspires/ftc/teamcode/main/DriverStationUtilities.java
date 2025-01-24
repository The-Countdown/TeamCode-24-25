package org.firstinspires.ftc.teamcode.main;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpModeManager;
import com.qualcomm.robotcore.eventloop.opmode.OpModeRegistrar;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.opmode.OpModeMeta;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

public class DriverStationUtilities {
    private static OpModeMeta metaForClass(String name) {
        return new OpModeMeta.Builder()
                .setName(name)
                .setGroup("Utilities")
                .setFlavor(OpModeMeta.Flavor.TELEOP)
                .build();
    }

    @OpModeRegistrar
    public static void register(OpModeManager manager) {
        manager.register(metaForClass("Switch Sides"), new SwitchSides());
    }

    static class SwitchSides extends LinearOpMode {
        @Override
        public void runOpMode() {
            while (!isStopRequested() && !isStarted()) {
                telemetry.addLine("Press the cross button to switch color for next match");
                telemetry.addData("Current color", Robot.color);
                telemetry.update();

                if (gamepad1.cross || gamepad2.cross) {
                    Robot.color = Robot.color == Robot.color.Blue ? Robot.color.Red : Robot.color.Blue;
                    while (gamepad1.cross || gamepad2.cross) {
                        idle();
                    }
                }
            }
        }
    }
}
