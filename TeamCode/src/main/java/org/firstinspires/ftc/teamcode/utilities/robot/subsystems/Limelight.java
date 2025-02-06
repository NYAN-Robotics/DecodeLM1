package org.firstinspires.ftc.teamcode.utilities.robot.subsystems;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.utilities.math.linearalgebra.Pose;
import org.firstinspires.ftc.teamcode.utilities.robot.RobotEx;


// 435/2 - 5 mm
public class Limelight implements Subsystem {

    Limelight3A limelight;

    Localizer odometry;

    Pose currentPose = new Pose(0, 0, 0);

    Telemetry telemetry;

    @Override
    public void onInit(HardwareMap newHardwareMap, Telemetry newTelemetry) {

        /*
        limelight = newHardwareMap.get(Limelight3A.class, "limelight");

        limelight.setPollRateHz(250);

        limelight.pipelineSwitch(0);


        telemetry = newTelemetry;

         */

        // Set Up limelight

    }

    @Override
    public void onOpmodeStarted() {
        if (limelight == null) return;
        limelight.start();
        odometry = RobotEx.getInstance().theLocalizer;
        // More setup
    }

    @Override
    public void onCyclePassed() {
        if (limelight == null) return;

        double angle = Math.toDegrees(odometry.getPose().getHeading());
        // limelight.updateRobotOrientation(0);

        Pose3D botpose = null;
        LLResult result = limelight.getLatestResult();

        if (result != null) {
            if (result.isValid()) {
                botpose = result.getBotpose_MT2(); // (x, y, z, h) -> (x, y, h)
            }
        }




        if (botpose != null) {
            Position botPosition = botpose.getPosition();
            currentPose = new Pose(botPosition.x * 39.37, botPosition.y * 39.37, odometry.getPose().getHeading());

        }


        telemetry.addLine("--LIMELIGHT--");
        telemetry.addData("X: ", currentPose.getX());
        telemetry.addData("Y: ", currentPose.getY());
        telemetry.addData("Heading: ", currentPose.getHeading());
        // Get the position of hte robot & store in a variable for access
    }

    public Pose getPose() {
        return currentPose;
    }
}
