package org.firstinspires.ftc.teamcode.utilities.robot.movement;

import static org.firstinspires.ftc.teamcode.utilities.robot.DriveConstants.A_MAX;
import static org.firstinspires.ftc.teamcode.utilities.robot.DriveConstants.K_A;
import static org.firstinspires.ftc.teamcode.utilities.robot.DriveConstants.K_V;
import static org.firstinspires.ftc.teamcode.utilities.robot.DriveConstants.MAX_CORRECTION_TIME;
import static org.firstinspires.ftc.teamcode.utilities.robot.DriveConstants.V_MAX;

import org.firstinspires.ftc.teamcode.utilities.robot.DriveConstants;

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

    public MovementConstants(
            double maxCorrectionTime
    ) {
        this(V_MAX, A_MAX, maxCorrectionTime, K_V, K_A);
    }

    public MovementConstants() {
        this(V_MAX, A_MAX, MAX_CORRECTION_TIME, K_V, K_A);
    }

    public MovementConstants(double velocityMax, double accelerationMax, double maxCorrectionTime) {
        this(velocityMax, accelerationMax, maxCorrectionTime, K_V, K_A);
    }
 }
