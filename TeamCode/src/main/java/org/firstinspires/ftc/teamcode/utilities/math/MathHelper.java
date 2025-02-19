package org.firstinspires.ftc.teamcode.utilities.math;

import org.firstinspires.ftc.teamcode.utilities.datastructures.Circle;
import org.firstinspires.ftc.teamcode.utilities.math.linearalgebra.Pose;

public class MathHelper {

    // Define a small epsilon value for precision comparison
    private static final double EPSILON = 1e-10;

    // Function to check if two doubles are approximately equal
    public static boolean epsilonEquals(double a, double b) {
        return Math.abs(a - b) < EPSILON;
    }

    public static double getDegreesErrorFromCamera(double centerCoordinate, double frameSize, double focalLength) {
        return Math.atan2((centerCoordinate - (frameSize / 2)), focalLength);
    }

    public static double truncate(double value, int decimalpoint) {

        // Using the pow() method
        value = value * Math.pow(10, decimalpoint);
        value = Math.floor(value);
        value = value / Math.pow(10, decimalpoint);

        return value;
    }

    public static <T extends Number> double lerp(T p0, T p1, double t) {

        return ((double) p0) * (1 - t) + ((double) p1) * t;
    }

    public static double clamp(double value, double min, double max) {
        return Math.min(Math.max(value, min), max);
    }


    public static double getErrorBetweenAngles(double targetAngle, double currentAngle) {
        double angle = AngleHelper.normDelta(targetAngle);
        double currentIMUPosition = AngleHelper.normDelta(currentAngle);

        double turnError = angle - currentIMUPosition;

        if (Math.abs(turnError) > Math.PI) {
            if (angle < 0) {
                angle = AngleHelper.norm(angle);
                turnError = angle - currentIMUPosition;
            } else if (angle > 0) {
                currentIMUPosition = AngleHelper.norm(currentIMUPosition);
                turnError = angle - currentIMUPosition;
            }
        }

        return turnError;
    }

    public static Circle findCircleFromPoints(Pose[] poses) {
        return findCircleFromPoints(poses[0], poses[1], poses[2]);
    }


    public static Circle findCircleFromPoints(Pose pose1, Pose pose2, Pose pose3) {

        double x1 = pose1.getX();
        double y1 = pose1.getY();

        double x2 = pose2.getX();
        double y2 = pose2.getY();

        double x3 = pose3.getX();
        double y3 = pose3.getY();

        double x12 = x1 - x2;
        double x13 = x1 - x3;

        double y12 = y1 - y2;
        double y13 = y1 - y3;

        double y31 = y3 - y1;
        double y21 = y2 - y1;

        double x31 = x3 - x1;
        double x21 = x2 - x1;

        // x1^2 - x3^2
        double sx13 = (int)(Math.pow(x1, 2) -
                Math.pow(x3, 2));

        // y1^2 - y3^2
        double sy13 = (int)(Math.pow(y1, 2) -
                Math.pow(y3, 2));

        double sx21 = (int)(Math.pow(x2, 2) -
                Math.pow(x1, 2));

        double sy21 = (int)(Math.pow(y2, 2) -
                Math.pow(y1, 2));

        double f = ((sx13) * (x12)
                + (sy13) * (x12)
                + (sx21) * (x13)
                + (sy21) * (x13))
                / (2 * ((y31) * (x12) - (y21) * (x13)));

        double g = ((sx13) * (y12)
                + (sy13) * (y12)
                + (sx21) * (y13)
                + (sy21) * (y13))
                / (2 * ((x31) * (y12) - (x21) * (y13)));

        double c = -(int)Math.pow(x1, 2) - (int)Math.pow(y1, 2) -
                2 * g * x1 - 2 * f * y1;

        // eqn of circle be x^2 + y^2 + 2*g*x + 2*f*y + c = 0
        // where centre is (h = -g, k = -f) and radius r
        // as r^2 = h^2 + k^2 - c
        double h = -g;
        double k = -f;
        double sqr_of_r = h * h + k * k - c;

        // r is the radius

        return new Circle(h, k, Math.sqrt(sqr_of_r));
    }

    // create a function that converts a veloicty in terms of in/sec to m/sec
    public static double imperialToMetricVelocity(double inchesPerSecond) {
        return inchesPerSecond * 0.0254;
    }

}