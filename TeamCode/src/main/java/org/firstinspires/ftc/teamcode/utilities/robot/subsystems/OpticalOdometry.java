package org.firstinspires.ftc.teamcode.utilities.robot.subsystems;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.utilities.math.linearalgebra.Pose;

public class OpticalOdometry implements Subsystem {

    SparkFunOTOS otos;

    Pose currentPose = new Pose();
    Pose lastPose = new Pose();

    @Override
    public void onInit(HardwareMap hardwareMap, Telemetry telemetry) {
        otos = hardwareMap.get(SparkFunOTOS.class, "sensor_otos");

        otos.setLinearUnit(DistanceUnit.INCH);
        otos.setAngularUnit(AngleUnit.DEGREES);


        /*

        SparkFunOTOS.Pose2D offset = new SparkFunOTOS.Pose2D(0, -3.75, -90);
        otos.setOffset(offset);
        otos.setLinearScalar(1.011);
        otos.setAngularScalar(0.992);

         */

        SparkFunOTOS.Pose2D offset = new SparkFunOTOS.Pose2D(-1.42, 0, 180);
        otos.setOffset(offset);
        otos.setLinearScalar(1);
        otos.setAngularScalar(1);

        otos.calibrateImu();

        otos.resetTracking();


    }

    @Override
    public void onOpmodeStarted() {

    }

    @Override
    public void onCyclePassed() {

        lastPose = currentPose;

        SparkFunOTOS.Pose2D pos = otos.getPosition();

        currentPose = new Pose(
                pos.x,
                pos.y,
                Math.toRadians(pos.h)
        );

    }

    public Pose getPose() {
        return currentPose;
    }

    public void setPose(Pose newPose) {
        otos.setPosition(new SparkFunOTOS.Pose2D(
                newPose.getX(),
                newPose.getY(),
                Math.toDegrees(newPose.getHeading())
        ));
    }
}
