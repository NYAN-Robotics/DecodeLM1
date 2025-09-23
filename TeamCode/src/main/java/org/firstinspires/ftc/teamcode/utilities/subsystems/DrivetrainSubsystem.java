package org.firstinspires.ftc.teamcode.utilities.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.utilities.config.hardware.cachedMotor;
import org.firstinspires.ftc.teamcode.utilities.config.core.robotConstants;
import org.firstinspires.ftc.teamcode.utilities.math.MotorMath;

/**
 * Drivetrain Subsystem - Handles two-motor drivetrain with PID velocity control
 */
public class DrivetrainSubsystem {

    // Hardware components
    private cachedMotor leftMotor;
    private cachedMotor rightMotor;

    // State variables
    private boolean motorsEnabled = false;
    private double leftTargetRPM = 0.0;
    private double rightTargetRPM = 0.0;
    private double currentTargetRPM = robotConstants.DEFAULT_TARGET_RPM;

    /**
     * Initialize the drivetrain hardware
     * @param hardwareMap The hardware map from the OpMode
     */
    public void init(HardwareMap hardwareMap) {
        // Initialize Left Motor
        DcMotorEx leftMotorEx = hardwareMap.get(DcMotorEx.class, robotConstants.LEFT_MOTOR_NAME);
        leftMotor = new cachedMotor(leftMotorEx);

        // Initialize Right Motor
        DcMotorEx rightMotorEx = hardwareMap.get(DcMotorEx.class, robotConstants.RIGHT_MOTOR_NAME);
        rightMotor = new cachedMotor(rightMotorEx);

        // Configure both motors for velocity control
        configureMotor(leftMotor);
        configureMotor(rightMotor);

        // Set initial state
        leftTargetRPM = 0.0;
        rightTargetRPM = 0.0;
        motorsEnabled = false;
    }

    /**
     * Configure a single motor for PID velocity control
     * @param motor The motor to configure
     */
    private void configureMotor(cachedMotor motor) {
        // Reset encoder and set run mode
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Set PID coefficients for Gobilda 6K motors
        motor.setPIDFCoefficients(
                robotConstants.MOTOR_KP,
                robotConstants.MOTOR_KI,
                robotConstants.MOTOR_KD,
                robotConstants.MOTOR_KF
        );
    }

    /**
     * Enable both motors at the current target RPM
     */
    public void enableMotors() {
        setBothMotorsRPM(currentTargetRPM);
        motorsEnabled = true;
    }

    /**
     * Disable both motors (set to 0 RPM)
     */
    public void disableMotors() {
        setBothMotorsRPM(0.0);
        motorsEnabled = false;
    }

    /**
     * Set both motors to the same RPM
     * @param rpm Target RPM for both motors
     */
    public void setBothMotorsRPM(double rpm) {
        setLeftMotorRPM(rpm);
        setRightMotorRPM(rpm);
        currentTargetRPM = rpm;
    }

    /**
     * Set left motor RPM
     * @param rpm Target RPM for left motor
     */
    public void setLeftMotorRPM(double rpm) {
        // Clamp RPM to safe bounds
        rpm = MotorMath.clampRPM(rpm, robotConstants.MIN_RPM, robotConstants.MAX_RPM);
        
        // Convert RPM to TPS and set velocity
        double targetTPS = MotorMath.gobildaRPMToTPS(rpm);
        leftMotor.setVelocity(targetTPS);
        leftTargetRPM = rpm;
        
        // Update enabled state
        motorsEnabled = (rpm > 0.0 || rightTargetRPM > 0.0);
    }

    /**
     * Set right motor RPM
     * @param rpm Target RPM for right motor
     */
    public void setRightMotorRPM(double rpm) {
        // Clamp RPM to safe bounds
        rpm = MotorMath.clampRPM(rpm, robotConstants.MIN_RPM, robotConstants.MAX_RPM);
        
        // Convert RPM to TPS and set velocity
        double targetTPS = MotorMath.gobildaRPMToTPS(rpm);
        rightMotor.setVelocity(targetTPS);
        rightTargetRPM = rpm;
        
        // Update enabled state
        motorsEnabled = (rpm > 0.0 || leftTargetRPM > 0.0);
    }

    /**
     * Increment both motors RPM by the configured increment
     */
    public void incrementRPM() {
        double newRPM = MotorMath.clampRPM(
                currentTargetRPM + robotConstants.RPM_INCREMENT,
                robotConstants.MIN_RPM,
                robotConstants.MAX_RPM
        );
        setBothMotorsRPM(newRPM);
    }

    /**
     * Decrement both motors RPM by the configured increment
     */
    public void decrementRPM() {
        double newRPM = MotorMath.clampRPM(
                currentTargetRPM - robotConstants.RPM_INCREMENT,
                robotConstants.MIN_RPM,
                robotConstants.MAX_RPM
        );
        setBothMotorsRPM(newRPM);
    }

    /**
     * Toggle motors on/off
     */
    public void toggleMotors() {
        if (motorsEnabled) {
            disableMotors();
        } else {
            enableMotors();
        }
    }

    /**
     * Get current left motor RPM
     * @return Current left motor RPM
     */
    public double getLeftMotorRPM() {
        return MotorMath.gobildaTPSToRPM(leftMotor.getVelocity());
    }

    /**
     * Get current right motor RPM
     * @return Current right motor RPM
     */
    public double getRightMotorRPM() {
        return MotorMath.gobildaTPSToRPM(rightMotor.getVelocity());
    }

    /**
     * Get target left motor RPM
     * @return Target left motor RPM
     */
    public double getLeftTargetRPM() {
        return leftTargetRPM;
    }

    /**
     * Get target right motor RPM
     * @return Target right motor RPM
     */
    public double getRightTargetRPM() {
        return rightTargetRPM;
    }

    /**
     * Get current target RPM (for both motors)
     * @return Current target RPM
     */
    public double getCurrentTargetRPM() {
        return currentTargetRPM;
    }

    /**
     * Check if motors are enabled
     * @return True if motors are enabled
     */
    public boolean areMotorsEnabled() {
        return motorsEnabled;
    }

    /**
     * Check if left motor is at target RPM
     * @return True if left motor is at target RPM
     */
    public boolean isLeftMotorAtTargetRPM() {
        double currentRPM = getLeftMotorRPM();
        return MotorMath.isAtTargetRPM(leftTargetRPM, currentRPM, robotConstants.RPM_TOLERANCE);
    }

    /**
     * Check if right motor is at target RPM
     * @return True if right motor is at target RPM
     */
    public boolean isRightMotorAtTargetRPM() {
        double currentRPM = getRightMotorRPM();
        return MotorMath.isAtTargetRPM(rightTargetRPM, currentRPM, robotConstants.RPM_TOLERANCE);
    }

    /**
     * Check if both motors are at target RPM
     * @return True if both motors are at target RPM
     */
    public boolean areMotorsAtTargetRPM() {
        return isLeftMotorAtTargetRPM() && isRightMotorAtTargetRPM();
    }

    /**
     * Get left motor position in ticks
     * @return Left motor position
     */
    public int getLeftMotorPosition() {
        return leftMotor.getCurrentPosition();
    }

    /**
     * Get right motor position in ticks
     * @return Right motor position
     */
    public int getRightMotorPosition() {
        return rightMotor.getCurrentPosition();
    }

    /**
     * Reset both motor encoders
     */
    public void resetEncoders() {
        leftMotor.resetEncoder();
        rightMotor.resetEncoder();
    }

    /**
     * Get telemetry data as a formatted string
     * @return Formatted telemetry string
     */
    public String getTelemetryData() {
        return String.format(
                "Motors: %s | L: %.0f/%.0f RPM | R: %.0f/%.0f RPM | Target: %.0f RPM",
                motorsEnabled ? "ON" : "OFF",
                getLeftMotorRPM(),
                leftTargetRPM,
                getRightMotorRPM(),
                rightTargetRPM,
                currentTargetRPM
        );
    }

    /**
     * Get detailed telemetry data
     * @return Detailed telemetry string
     */
    public String getDetailedTelemetryData() {
        return String.format(
                "L Motor: %.0f/%.0f RPM (%.0f TPS) | R Motor: %.0f/%.0f RPM (%.0f TPS) | Target: %.0f RPM",
                getLeftMotorRPM(),
                leftTargetRPM,
                leftMotor.getVelocity(),
                getRightMotorRPM(),
                rightTargetRPM,
                rightMotor.getVelocity(),
                currentTargetRPM
        );
    }
}
