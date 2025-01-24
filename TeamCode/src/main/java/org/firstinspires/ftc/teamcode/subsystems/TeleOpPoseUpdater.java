package org.firstinspires.ftc.teamcode.subsystems;

import java.util.Date;

public class TeleOpPoseUpdater implements Runnable {
    @Override
    public void run() {
        while (Robot.rb.opMode.opModeIsActive()) {
            Robot.teleOpStart = Robot.rb.roadRunner.pose;
            Robot.teleOpStartDate = new Date();
        }
    }
}
