package org.firstinspires.ftc.teamcode.utilities.config.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

/**
 * Cached Motor Wrapper - Provides enhanced motor control with caching and PID support
 */
public class cachedMotor {

    private final DcMotorEx motor;
    private double lastPower = 0.0;
    private double lastVelocity = 0.0;
    private int lastTargetPosition = 0;
    private DcMotor.RunMode lastRunMode;
    private boolean velocityControlEnabled = false;

    /**
     * Constructor
     * @param motor The DcMotorEx instance to wrap
     */
    public cachedMotor(DcMotorEx motor) {
        this.motor = motor;
        this.lastRunMode = motor.getMode();
    }

    /**
     * Set motor power with caching to avoid unnecessary hardware calls
     * @param power Motor power (-1.0 to 1.0)
     */
    public void setPower(double power) {
        if (Math.abs(power - lastPower) > 0.001) {
            motor.setPower(power);
            lastPower = power;
            velocityControlEnabled = false;
        }
    }

    /**
     * Set target velocity for PID control
     * @param velocity Target velocity in ticks per second
     */
    public void setVelocity(double velocity) {
        if (Math.abs(velocity - lastVelocity) > 0.1) {
            motor.setVelocity(velocity);
            lastVelocity = velocity;
            velocityControlEnabled = true;
        }
    }

    /**
     * Set target position for position control
     * @param targetPosition Target position in ticks
     */
    public void setTargetPosition(int targetPosition) {
        if (targetPosition != lastTargetPosition) {
            motor.setTargetPosition(targetPosition);
            lastTargetPosition = targetPosition;
        }
    }

    /**
     * Set run mode with caching
     * @param runMode The desired run mode
     */
    public void setMode(DcMotor.RunMode runMode) {
        if (runMode != lastRunMode) {
            motor.setMode(runMode);
            lastRunMode = runMode;
        }
    }

    /**
     * Set zero power behavior
     * @param zeroPowerBehavior The desired zero power behavior
     */
    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
        motor.setZeroPowerBehavior(zeroPowerBehavior);
    }

    /**
     * Set PIDF coefficients for velocity control
     * @param kP Proportional coefficient
     * @param kI Integral coefficient
     * @param kD Derivative coefficient
     * @param kF Feed-forward coefficient
     */
    public void setPIDFCoefficients(double kP, double kI, double kD, double kF) {
        PIDFCoefficients pidfCoeffs = new PIDFCoefficients(kP, kI, kD, kF);
        motor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoeffs);
    }

    /**
     * Get current motor position
     * @return Current position in ticks
     */
    public int getCurrentPosition() {
        return motor.getCurrentPosition();
    }

    /**
     * Get current motor velocity
     * @return Current velocity in ticks per second
     */
    public double getVelocity() {
        return motor.getVelocity();
    }

    /**
     * Get current motor power
     * @return Current power setting
     */
    public double getPower() {
        return motor.getPower();
    }

    /**
     * Check if motor is at target position (for RUN_TO_POSITION mode)
     * @return True if at target position
     */
    public boolean isAtTargetPosition() {
        return !motor.isBusy();
    }

    /**
     * Check if velocity control is currently enabled
     * @return True if velocity control is active
     */
    public boolean isVelocityControlEnabled() {
        return velocityControlEnabled;
    }

    /**
     * Get the underlying motor instance (use sparingly)
     * @return The DcMotorEx instance
     */
    public DcMotorEx getMotor() {
        return motor;
    }

    /**
     * Reset encoder position to zero
     */
    public void resetEncoder() {
        DcMotor.RunMode currentMode = motor.getMode();
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(currentMode);
        lastRunMode = currentMode;
    }
}