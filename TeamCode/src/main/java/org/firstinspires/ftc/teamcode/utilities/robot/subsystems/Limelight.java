package org.firstinspires.ftc.teamcode.utilities.robot.subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.utilities.math.linearalgebra.Pose;
import org.firstinspires.ftc.teamcode.utilities.robot.RobotEx;


// 36 mm
public class Limelight implements Subsystem {

    Limelight3A limelight;

    OpticalOdometry odometry;

    Pose currentPose;

    @Override
    public void onInit(HardwareMap hardwareMap, Telemetry telemetry) {

        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        limelight.setPollRateHz(100);

        limelight.pipelineSwitch(0);

        limelight.start();

        // Set Up limelight

    }

    @Override
    public void onOpmodeStarted() {
        // if (limelight == null) return;
        odometry = RobotEx.getInstance().odometry;
        // More setup
    }

    @Override
    public void onCyclePassed() {
        if (limelight == null) return;

        limelight.updateRobotOrientation(Math.toRadians(odometry.getPose().getHeading()));

        Pose3D botpose = null;
        LLResult result = limelight.getLatestResult();
        if (result != null) {
            if (result.isValid()) {
                botpose = result.getBotpose(); // (x, y, z, h) -> (x, y, h)
                telemetry.addData("tx", result.getTx());
                telemetry.addData("ty", result.getTy());
                telemetry.addData("Botpose", botpose.toString());
            }
        }


        if (botpose != null) {
            Position botPosition = botpose.getPosition();
            currentPose = new Pose(botPosition.x, botPosition.y, odometry.getPose().getHeading());
        }
        // Get the position of hte robot & store in a variable for access
    }

    public Pose getPose() {
        return currentPose;
    }
}
