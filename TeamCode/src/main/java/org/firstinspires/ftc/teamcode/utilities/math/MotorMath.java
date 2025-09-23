package org.firstinspires.ftc.teamcode.utilities.math;

/**
 * Motor Math Utilities - Contains calculations for motor control and PID
 */
public class MotorMath {
    
    /**
     * Convert RPM to Ticks Per Second (TPS)
     * @param rpm Revolutions per minute
     * @param tpr Ticks per revolution
     * @return Ticks per second
     */
    public static double rpmToTPS(double rpm, double tpr) {
        return tpr * (rpm / 60.0);
    }
    
    /**
     * Convert Ticks Per Second (TPS) to RPM
     * @param tps Ticks per second
     * @param tpr Ticks per revolution
     * @return Revolutions per minute
     */
    public static double tpsToRPM(double tps, double tpr) {
        return (tps / tpr) * 60.0;
    }
    
    /**
     * Convert RPM to TPS using Gobilda 6K motor specifications
     * @param rpm Target RPM
     * @return Ticks per second
     */
    public static double gobildaRPMToTPS(double rpm) {
        return rpmToTPS(rpm, 28.0); // Gobilda 6K has 28 TPR
    }
    
    /**
     * Convert TPS to RPM using Gobilda 6K motor specifications
     * @param tps Ticks per second
     * @return RPM
     */
    public static double gobildaTPSToRPM(double tps) {
        return tpsToRPM(tps, 28.0); // Gobilda 6K has 28 TPR
    }
    
    /**
     * Clamp RPM value within safe bounds
     * @param rpm RPM value to clamp
     * @param minRPM Minimum allowed RPM
     * @param maxRPM Maximum allowed RPM
     * @return Clamped RPM value
     */
    public static double clampRPM(double rpm, double minRPM, double maxRPM) {
        return Math.max(minRPM, Math.min(maxRPM, rpm));
    }
    
    /**
     * Clamp TPS value within safe bounds
     * @param tps TPS value to clamp
     * @param minTPS Minimum allowed TPS
     * @param maxTPS Maximum allowed TPS
     * @return Clamped TPS value
     */
    public static double clampTPS(double tps, double minTPS, double maxTPS) {
        return Math.max(minTPS, Math.min(maxTPS, tps));
    }
    
    /**
     * Calculate RPM error for PID control
     * @param targetRPM Target RPM
     * @param currentRPM Current RPM
     * @return RPM error
     */
    public static double calculateRPMError(double targetRPM, double currentRPM) {
        return targetRPM - currentRPM;
    }
    
    /**
     * Check if motor is at target RPM within tolerance
     * @param targetRPM Target RPM
     * @param currentRPM Current RPM
     * @param tolerance RPM tolerance
     * @return True if within tolerance
     */
    public static boolean isAtTargetRPM(double targetRPM, double currentRPM, double tolerance) {
        return Math.abs(calculateRPMError(targetRPM, currentRPM)) <= tolerance;
    }
    
    /**
     * Calculate velocity error for PID control
     * @param targetTPS Target TPS
     * @param currentTPS Current TPS
     * @return TPS error
     */
    public static double calculateVelocityError(double targetTPS, double currentTPS) {
        return targetTPS - currentTPS;
    }
    
    /**
     * Check if motor is at target velocity within tolerance
     * @param targetTPS Target TPS
     * @param currentTPS Current TPS
     * @param tolerance TPS tolerance
     * @return True if within tolerance
     */
    public static boolean isAtTargetVelocity(double targetTPS, double currentTPS, double tolerance) {
        return Math.abs(calculateVelocityError(targetTPS, currentTPS)) <= tolerance;
    }
    
    /**
     * Calculate power percentage from RPM (for basic control)
     * @param targetRPM Target RPM
     * @param maxRPM Maximum RPM of motor
     * @return Power percentage (0.0 to 1.0)
     */
    public static double rpmToPower(double targetRPM, double maxRPM) {
        return Math.max(0.0, Math.min(1.0, targetRPM / maxRPM));
    }
    
    /**
     * Calculate RPM from power percentage (for basic control)
     * @param power Power percentage (0.0 to 1.0)
     * @param maxRPM Maximum RPM of motor
     * @return Estimated RPM
     */
    public static double powerToRPM(double power, double maxRPM) {
        return power * maxRPM;
    }
}