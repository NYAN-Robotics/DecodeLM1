package org.firstinspires.ftc.teamcode.utilities.math;

public class MotorMath {
    public static double rpmToTPS(double rpm, double tpr) {
        return tpr * (rpm / 60);
    }
}