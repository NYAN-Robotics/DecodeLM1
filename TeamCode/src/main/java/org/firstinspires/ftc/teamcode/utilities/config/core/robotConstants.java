package org.firstinspires.ftc.teamcode.utilities.config.core;

import org.firstinspires.ftc.teamcode.utilities.math.MotorMath;

/**
 * Robot Constants - Central location for all robot configuration values
 */
public class robotConstants {

    // ========== HARDWARE CONSTANTS ==========

    // Motor Names (configured in Robot Configuration)
    public static final String LEFT_MOTOR_NAME = "leftMotor";
    public static final String RIGHT_MOTOR_NAME = "rightMotor";

    // Servo Names
    public static final String TEST_SERVO_NAME = "testServo";

    // Sensor Names
    public static final String LIMELIGHT_NAME = "limelight"; // For future use

    // ========== MOTOR SPECIFICATIONS ==========
    
    // Gobilda 6K RPM Motor Specifications
    public static final double GOBILDA_6K_RPM = 6000.0; // Maximum RPM
    public static final double GOBILDA_6K_TPR = 28.0; // Ticks per revolution (1:1 gear ratio)
    public static final double GOBILDA_6K_MAX_VELOCITY = MotorMath.rpmToTPS(GOBILDA_6K_RPM, GOBILDA_6K_TPR);

    // ========== PID MOTOR CONSTANTS ==========

    // PID Coefficients for velocity control (tuned for Gobilda 6K)
    public static final double MOTOR_KP = 15.0;
    public static final double MOTOR_KI = 2.0;
    public static final double MOTOR_KD = 1.0;
    public static final double MOTOR_KF = 14.0;

    // ========== DRIVETRAIN CONSTANTS ==========
    
    // Default RPM settings for teleop control
    public static final double DEFAULT_TARGET_RPM = 1000.0; // Default 1000 RPM
    public static final double MIN_RPM = 0.0;
    public static final double MAX_RPM = 3000.0; // Conservative max for safety
    public static final double RPM_INCREMENT = 100.0; // RPM change per input
    
    // Velocity control
    public static final double DEFAULT_TARGET_VELOCITY = MotorMath.rpmToTPS(DEFAULT_TARGET_RPM, GOBILDA_6K_TPR);

    // ========== SERVO CONSTANTS ==========

    // Servo positions (0.0 to 1.0)
    public static final double SERVO_MIN_POSITION = 0.0;
    public static final double SERVO_MAX_POSITION = 1.0;
    public static final double SERVO_HOME_POSITION = 0.5;
    public static final double SERVO_INCREMENT = 0.1;

    // ========== LIMELIGHT CONSTANTS (Future Use) ==========

    // Limelight pipeline numbers
    public static final int LIMELIGHT_PIPELINE_APRILTAG = 0;
    public static final int LIMELIGHT_PIPELINE_GAME_PIECE = 1;

    // Vision processing constants
    public static final double VISION_TARGET_HEIGHT = 12.0; // inches
    public static final double CAMERA_HEIGHT = 8.0; // inches
    public static final double CAMERA_ANGLE = 15.0; // degrees

    // ========== OPERATIONAL CONSTANTS ==========

    // Timeouts and delays
    public static final double MOTOR_TIMEOUT_SECONDS = 5.0;
    public static final double SERVO_MOVE_TIME_MS = 500;

    // Control loop periods
    public static final double CONTROL_LOOP_PERIOD_MS = 20;

    // Tolerances
    public static final double MOTOR_VELOCITY_TOLERANCE = 50.0; // ticks/sec (increased for higher RPM motors)
    public static final double SERVO_POSITION_TOLERANCE = 0.05;
    public static final double RPM_TOLERANCE = 50.0; // RPM tolerance for PID control
}