package org.firstinspires.ftc.teamcode.utilities.math.linearalgebra;

import java.util.function.Function;

public class Pose {
    public double theX;
    public double theY;
    public double theHeading;

    public Pose() {
        this(0, 0, 0);
    }

    public Pose(double aX, double aY, double aHeading) {
        theX = aX;
        theY = aY;
        theHeading = aHeading;
    }

    public Pose(Pose other) {
        theX = other.theX;
        theY = other.theY;
        theHeading = other.theHeading;
    }

    public double getX() {
        return theX;
    }

    public void setX(double aX) {
        theX = aX;
    }

    public double getY() {
        return theY;
    }

    public void setY(double aY) {
        theY = aY;
    }

    public double getHeading() {
        return theHeading;
    }

    public void setHeading(double aHeading) {
        this.theHeading = aHeading;
    }

    public void add(Pose other) {
        theX += other.getX();
        theY += other.getY();
        theHeading += other.getHeading();
    }

    public void subtract(Pose other) {
        theX -= other.getX();
        theY -= other.getY();
        theHeading -= other.getHeading();
    }


    public void rotated(double theAngle) {
        double x = theX;
        double y = theY;

        theX = y * Math.cos(theAngle) - x * Math.sin(theAngle);
        theY = y * Math.sin(theAngle) + x * Math.cos(theAngle);
        theHeading += theAngle;
    }

    public Pose times(double other) {
        theX *= other;
        theY *= other;

        return this;
    }

    public Pose abs() {
        theX = Math.abs(theX);
        theY = Math.abs(theY);
        theHeading = Math.abs(theHeading);

        return this;
    }

    public Pose map(Function<Double, Double> func) {
        theX = func.apply(theX);
        theY = func.apply(theY);
        theHeading = func.apply(theHeading);

        return this;
    }

    public boolean lessThan(Pose other) {
        return (theX < other.getX()) && (theY < other.getY()) && (theHeading < other.getHeading());
    }

    public double magnitude() {
        return Math.sqrt(theX * theX + theY * theY);
    }

    public double distance(Pose other) {
        return new Pose(
                theX - other.getX(),
                theY - other.getY(),
                theHeading - other.getHeading()
        ).magnitude();
    }
}
