package org.firstinspires.ftc.teamcode.utilities.robot.subsystems;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.utilities.datastructures.CentripetalBuffer;
import org.firstinspires.ftc.teamcode.utilities.drivers.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.utilities.math.linearalgebra.Pose;
import org.opencv.core.Mat;

public class PinpointOdometry extends Localizer {

    GoBildaPinpointDriver pinpoint;

    Telemetry telemetry;

    ElapsedTime poseTimer;


    @Override
    public void onInit(HardwareMap hardwareMap, Telemetry telemetry) {

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        pinpoint.setOffsets(-127, 41.075);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        pinpoint.resetPosAndIMU();

        this.telemetry = telemetry;

        poseTimer = new ElapsedTime();
    }

    @Override
    public void onOpmodeStarted() {

    }

    @Override
    public void onCyclePassed() {

        pinpoint.update();

        lastPose = currentPose;
        lastVelocity = currentVelocity;

        Pose2D pos = pinpoint.getPosition();

        Pose2D vel = pinpoint.getVelocity();

        if (pos == null || vel == null) {
            System.out.println("NULL POSE!!!: ");
            return;
        }

        currentPose = new Pose(
                pos.getX(DistanceUnit.INCH),
                -pos.getY(DistanceUnit.INCH),
                pos.getHeading(AngleUnit.RADIANS) + Math.PI / 4
        );


        currentVelocity = new Pose(
                vel.getX(DistanceUnit.INCH),
                vel.getY(DistanceUnit.INCH),
                vel.getHeading(AngleUnit.RADIANS)
        );

        telemetry.addLine("---Pinpoint---");

        telemetry.addData("x pos: ", currentPose.getX());
        telemetry.addData("y pos: ", currentPose.getY());
        telemetry.addData("heading: ", currentPose.getHeading());

        telemetry.addData("x vel: ", currentVelocity.getX());
        telemetry.addData("y vel: ", currentVelocity.getY());

        telemetry.addData("heading vel", currentVelocity.getHeading());
        telemetry.addData("frequency", pinpoint.getFrequency());

        telemetry.addLine("-------------");

        poseTimer.reset();

        centripetalPoseBuffer.addPose(currentPose);
    }

    public void setPose(Pose newPose) {
        pinpoint.setPosition(
                new Pose2D(
                        DistanceUnit.INCH,
                        newPose.getX(),
                        newPose.getY(),
                        AngleUnit.RADIANS,
                        newPose.getHeading() - Math.PI / 2
                )
        );
    }
}
