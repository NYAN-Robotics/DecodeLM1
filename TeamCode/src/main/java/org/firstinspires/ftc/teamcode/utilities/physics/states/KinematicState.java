package org.firstinspires.ftc.teamcode.utilities.physics.states;

import org.firstinspires.ftc.teamcode.utilities.math.linearalgebra.Pose;

public class KinematicState {

    Pose thePose;
    double theVelocity;
    double theAcceleration;

    public KinematicState(Pose aPose, double aVelocity, double aAcceleration) {
        thePose = aPose;
        theVelocity = aVelocity;
        theAcceleration = aAcceleration;
    }

    public Pose getPose() {
        return thePose;
    }

    public double getVelocity() {
        return theVelocity;
    }

    public double getAcceleration() {
        return theAcceleration;
    }
}
