package org.firstinspires.ftc.teamcode.utilities.config.core;

import org.firstinspires.ftc.teamcode.utilities.math.MotorMath;

/**
 * Robot Constants - Central location for all robot configuration values
 */
public class robotConstants {

    // ========== HARDWARE CONSTANTS ==========

    // Motor Names (configured in Robot Configuration)
    public static final String PID_MOTOR_NAME = "pidMotor";

    // Servo Names
    public static final String TEST_SERVO_NAME = "testServo";

    // Sensor Names
    public static final String LIMELIGHT_NAME = "limelight"; // For future use

    // ========== PID MOTOR CONSTANTS ==========

    // PID Coefficients for velocity control
    public static final double MOTOR_KP = 10.0;
    public static final double MOTOR_KI = 3.0;
    public static final double MOTOR_KD = 0.5;
    public static final double MOTOR_KF = 12.0;

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

    // TPR
    public static final double YJ_5203_TPR = 537.7;

    // Control loop periods
    public static final double CONTROL_LOOP_PERIOD_MS = 20;

    // Tolerances
    public static final double MOTOR_VELOCITY_TOLERANCE = 10.0; // ticks/sec
    public static final double SERVO_POSITION_TOLERANCE = 0.05;

    // Motor specifications
    public static final double MOTOR_MAX_RPM = 312.0;
    public static final double MOTOR_TARGET_VELOCITY = MotorMath.rpmToTPS(60, YJ_5203_TPR); // Target velocity in ticks/sec
}