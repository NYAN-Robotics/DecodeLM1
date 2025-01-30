package org.firstinspires.ftc.teamcode.utilities.robot.subsystems;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;

import org.firstinspires.ftc.teamcode.utilities.datastructures.CentripetalBuffer;
import org.firstinspires.ftc.teamcode.utilities.math.linearalgebra.Pose;

public abstract class Localizer implements Subsystem {

    public CentripetalBuffer centripetalPoseBuffer = new CentripetalBuffer(3);

    Pose currentPose = new Pose();
    Pose lastPose = new Pose();

    Pose currentVelocity = new Pose();
    Pose lastVelocity = new Pose();

    public Pose getPose() {
        return currentPose;
    }

    public Pose getLastPose() {
        return lastPose;
    }

    public Pose getVelocity() {
        return currentVelocity;
    }

    public Pose getLastVelocity() {
        return lastVelocity;
    }

    public abstract void setPose(Pose newPose);

}
