package org.firstinspires.ftc.teamcode.utilities.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.utilities.config.hardware.cachedMotor;
import org.firstinspires.ftc.teamcode.utilities.config.core.robotConstants;

/**
 * Test Bench Subsystem - Handles PID motor and servo control
 */
public class TestBenchSubsystem {

    // Hardware components
    private CachedMotor pidMotor;
    private Servo testServo;

    // State variables
    private boolean motorEnabled = false;
    private double currentServoPosition = robotConstants.SERVO_HOME_POSITION;

    /**
     * Initialize the test bench hardware
     * @param hardwareMap The hardware map from the OpMode
     */
    public void init(HardwareMap hardwareMap) {
        // Initialize PID Motor
        DcMotorEx motorEx = hardwareMap.get(DcMotorEx.class, robotConstants.PID_MOTOR_NAME);
        pidMotor = new cachedMotor(motorEx);

        // Configure motor for velocity control
        pidMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        pidMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        pidMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Set PID coefficients
        pidMotor.setPIDFCoefficients(
                robotConstants.MOTOR_KP,
                robotConstants.MOTOR_KI,
                robotConstants.MOTOR_KD,
                robotConstants.MOTOR_KF
        );

        // Initialize Servo
        testServo = hardwareMap.get(Servo.class, robotConstants.TEST_SERVO_NAME);
        testServo.setPosition(robotConstants.SERVO_HOME_POSITION);
        currentServoPosition = robotConstants.SERVO_HOME_POSITION;
    }

    /**
     * Enable PID motor at target velocity
     */
    public void enableMotor() {
        pidMotor.setVelocity(robotConstants.MOTOR_TARGET_VELOCITY);
        motorEnabled = true;
    }

    /**
     * Disable PID motor
     */
    public void disableMotor() {
        pidMotor.setPower(0.0);
        motorEnabled = false;
    }

    /**
     * Set motor velocity directly
     * @param velocity Target velocity in ticks per second
     */
    public void setMotorVelocity(double velocity) {
        pidMotor.setVelocity(velocity);
        motorEnabled = (velocity != 0.0);
    }

    /**
     * Toggle motor on/off
     */
    public void toggleMotor() {
        if (motorEnabled) {
            disableMotor();
        } else {
            enableMotor();
        }
    }

    /**
     * Set servo position
     * @param position Servo position (0.0 to 1.0)
     */
    public void setServoPosition(double position) {
        position = Math.max(robotConstants.SERVO_MIN_POSITION,
                Math.min(robotConstants.SERVO_MAX_POSITION, position));

        testServo.setPosition(position);
        currentServoPosition = position;
    }

    /**
     * Increment servo position
     */
    public void incrementServoPosition() {
        double newPosition = currentServoPosition + robotConstants.SERVO_INCREMENT;
        setServoPosition(newPosition);
    }

    /**
     * Decrement servo position
     */
    public void decrementServoPosition() {
        double newPosition = currentServoPosition - robotConstants.SERVO_INCREMENT;
        setServoPosition(newPosition);
    }

    /**
     * Move servo to home position
     */
    public void homeServo() {
        setServoPosition(robotConstants.SERVO_HOME_POSITION);
    }

    /**
     * Get current motor velocity
     * @return Current velocity in ticks per second
     */
    public double getMotorVelocity() {
        return pidMotor.getVelocity();
    }

    /**
     * Get target motor velocity
     * @return Target velocity in ticks per second
     */
    public double getTargetVelocity() {
        return motorEnabled ? robotConstants.MOTOR_TARGET_VELOCITY : 0.0;
    }

    /**
     * Get current servo position
     * @return Current servo position (0.0 to 1.0)
     */
    public double getServoPosition() {
        return currentServoPosition;
    }

    /**
     * Check if motor is enabled
     * @return True if motor is enabled
     */
    public boolean isMotorEnabled() {
        return motorEnabled;
    }

    /**
     * Get motor position in ticks
     * @return Current motor position
     */
    public int getMotorPosition() {
        return pidMotor.getCurrentPosition();
    }

    /**
     * Check if motor velocity is within tolerance of target
     * @return True if motor is at target velocity
     */
    public boolean isMotorAtTargetVelocity() {
        if (!motorEnabled) return true;

        double currentVelocity = getMotorVelocity();
        double targetVelocity = getTargetVelocity();
        return Math.abs(currentVelocity - targetVelocity) <= robotConstants.MOTOR_VELOCITY_TOLERANCE;
    }

    /**
     * Get telemetry data as a formatted string
     * @return Formatted telemetry string
     */
    public String getTelemetryData() {
        return String.format(
                "Motor: %s | Vel: %.1f/%.1f | Pos: %d | Servo: %.2f",
                motorEnabled ? "ON" : "OFF",
                getMotorVelocity(),
                getTargetVelocity(),
                getMotorPosition(),
                currentServoPosition
        );
    }
}