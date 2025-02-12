package org.firstinspires.ftc.teamcode.utilities.robot.subsystems;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.utilities.math.linearalgebra.Pose;

public class OpticalOdometry extends Localizer {

    SparkFunOTOS otos;


    Telemetry telemetry;

    ElapsedTime poseTimer;

    @Override
    public void onInit(HardwareMap hardwareMap, Telemetry telemetry) {
        otos = hardwareMap.get(SparkFunOTOS.class, "otos_sensor");

        otos.setLinearUnit(DistanceUnit.INCH);
        otos.setAngularUnit(AngleUnit.DEGREES);


        /*

        SparkFunOTOS.Pose2D offset = new SparkFunOTOS.Pose2D(0, -3.75, -90);
        otos.setOffset(offset);
        otos.setLinearScalar(1.011);
        otos.setAngularScalar(0.992);

         */

        // SparkFunOTOS.Pose2D offset = new SparkFunOTOS.Pose2D(-1.42, 0, 180);
        SparkFunOTOS.Pose2D offset = new SparkFunOTOS.Pose2D(-1.42, 0, 270);
        otos.setOffset(offset);
        otos.setLinearScalar(1);
        otos.setAngularScalar(0.992); // -23.503

        otos.calibrateImu();

        otos.resetTracking();

        this.telemetry = telemetry;

        poseTimer = new ElapsedTime();

    }

    @Override
    public void onOpmodeStarted() {

    }

    @Override
    public void onCyclePassed() {

        lastPose = currentPose;

        SparkFunOTOS.Pose2D pos = otos.getPosition();
        // SparkFunOTOS.Pose2D vel = otos.getVelocity();

        currentPose = new Pose(
                pos.x,
                pos.y,
                Math.toRadians(pos.h)
        );

        currentVelocity = new Pose(
                (currentPose.getX() - lastPose.getX()) / poseTimer.seconds(),
                (currentPose.getY() - lastPose.getY()) / poseTimer.seconds(),
                (currentPose.getHeading() - lastPose.getHeading()) / poseTimer.seconds()
        );


        telemetry.addData("x: ", currentPose.getX());
        telemetry.addData("y: ", currentPose.getY());
        // telemetry.addData("x velocity: ", vel.x);
        // telemetry.addData("y velocity: ", vel.y);

        telemetry.addData("h: ", currentPose.getHeading());

        poseTimer.reset();

    }

    public void setPose(Pose newPose) {
        otos.setPosition(new SparkFunOTOS.Pose2D(
                newPose.getX(),
                newPose.getY(),
                Math.toDegrees(newPose.getHeading())
        ));
    }
}
