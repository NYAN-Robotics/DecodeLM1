package org.firstinspires.ftc.teamcode.utilities.robot.movement;

import org.firstinspires.ftc.teamcode.utilities.math.linearalgebra.Pose;

public class MovementStateCommand {
    Pose thePose;
    double feedforwardX;
    double feedforwardY;
    
    public MovementStateCommand(Pose aPose, double aFeedforwardX, double aFeedforwardY) {
        thePose = aPose;
        feedforwardX = aFeedforwardX;
        feedforwardY = aFeedforwardY;
    }
    
    public Pose getPose() {
        return thePose;
    }
    
    public double getFeedforwardX() {
        return feedforwardX;
    }
    
    public double getFeedforwardY() {
        return feedforwardY;
    }
    
}
