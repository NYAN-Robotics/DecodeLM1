package org.firstinspires.ftc.teamcode.utilities.robot.movement;

public class MovementConstants {

    public double velocityMax;
    public double accelerationMax;
    public double maxCorrectionTime;
    public double kV;
    public double kA;

    public MovementConstants(
            double velocityMax,
            double accelerationMax,
            double maxCorrectionTime,
            double kV,
            double kA
    ) {
        this.velocityMax = velocityMax;
        this.accelerationMax = accelerationMax;
        this.maxCorrectionTime = maxCorrectionTime;
        this.kV = kV;
        this.kA = kA;
    }
}
