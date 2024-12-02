package org.firstinspires.ftc.teamcode.utilities.controltheory;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.utilities.controltheory.feedback.GeneralPIDController;
import org.firstinspires.ftc.teamcode.utilities.controltheory.motionprofiler.MotionProfile;

public class MotionProfiledMotion {

    GeneralPIDController feedbackPositionController;
    public MotionProfile feedforwardProfile;

    public ElapsedTime timer;

    private double kV = 0;
    private double kA = 0;

    private double vMax = 1000;
    private double aMax = 1000;
    public MotionProfiledMotion(MotionProfile newFeedforwardProfile, GeneralPIDController newPositionController) {
        this.feedforwardProfile = newFeedforwardProfile;
        this.feedbackPositionController = newPositionController;

        this.timer = new ElapsedTime();
    }

    public double getOutput(double currentPosition) {

        double currentUpdateTime = timer.seconds();

        double targetPosition = feedforwardProfile.getPositionFromTime(currentUpdateTime);
        double targetVelocity = feedforwardProfile.getVelocityFromTime(currentUpdateTime);
        double targetAcceleration = feedforwardProfile.getAccelerationFromTime(currentUpdateTime);

        double feedforward = targetAcceleration * kA + targetVelocity * kV;

        double feedback = feedbackPositionController.getOutputFromError(
                targetPosition - currentPosition
        );


        return feedback + feedforward;
    }

    public void setPIDCoefficients(double kP, double kI, double kD, double kF) {
        feedbackPositionController.updateCoefficients(
                kP, kI, kD, kF
        );
    }

    public void setProfileCoefficients(double newKV, double newKA, double newVMax, double newAMax) {
        kV = newKV;
        kA = newKA;
        vMax = newVMax;
        aMax = newAMax;
    }

    public void updateTargetPosition(double newTargetPosition, double newCurrentPosition) {
        feedforwardProfile = new MotionProfile(
                newCurrentPosition,
                newTargetPosition,
                vMax,
                aMax
        );

        timer.reset();
    }

    public boolean atTargetPosition() {
        return timer.seconds() >= feedforwardProfile.getDuration();
    }

}
