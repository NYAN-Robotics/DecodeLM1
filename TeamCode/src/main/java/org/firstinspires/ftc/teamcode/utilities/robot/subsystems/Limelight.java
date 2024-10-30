package org.firstinspires.ftc.teamcode.utilities.robot.subsystems;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.utilities.robot.RobotEx;


// 36 mm
public class Limelight implements Subsystem {

    Limelight3A limelight;

    OpticalOdometry odometry;

    @Override
    public void onInit(HardwareMap hardwareMap, Telemetry telemetry) {

        /*limelight = hardwareMap.get(Limelight3A.class, "limelight");

        limelight.setPollRateHz(250);
        limelight.start();

         */
    }

    @Override
    public void onOpmodeStarted() {
        if (limelight == null) return;

        odometry = RobotEx.getInstance().odometry;
    }

    @Override
    public void onCyclePassed() {
        if (limelight == null) return;

        limelight.updateRobotOrientation(Math.toRadians(odometry.getPose().getHeading()));
    }
}
