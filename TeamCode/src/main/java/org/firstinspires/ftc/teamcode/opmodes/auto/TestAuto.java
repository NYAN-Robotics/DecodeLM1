package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.utilities.subsystems.TestBenchSubsystem;

/**
 * Test Autonomous OpMode - Automated testing of PID motor and servo functionality
 *
 * Test Sequence:
 * 1. Initialize hardware
 * 2. Test servo positioning (home -> max -> min -> home)
 * 3. Test motor PID control at different velocities
 * 4. Test motor position control
 * 5. Clean shutdown
 */
@Autonomous(name="Test Bench Auto", group="Test")
public class TestAuto extends LinearOpMode {

    // Subsystems
    private TestBenchSubsystem testBench;

    // Timing
    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime testTimer = new ElapsedTime();

    // Test parameters
    private static final double SERVO_TEST_DELAY_MS = 1000;
    private static final double MOTOR_TEST_DELAY_MS = 2000;
    private static final double VELOCITY_TEST_1 = 100.0; // ticks/sec
    private static final double VELOCITY_TEST_2 = 300.0; // ticks/sec
    private static final double VELOCITY_TEST_3 = 500.0; // ticks/sec

    @Override
    public void runOpMode() {
        // Initialize subsystem
        testBench = new TestBenchSubsystem();
        testBench.init(hardwareMap);

        // Reset timers
        runtime.reset();
        testTimer.reset();

        telemetry.addData("Status", "Initialized - Ready for Testing");
        telemetry.addData("Test Sequence", "Servo -> Motor Velocity -> Motor Position");
        telemetry.update();

        // Wait for start
        waitForStart();
        runtime.reset();

        // Run test sequence
        runTestSequence();

        // Final status
        telemetry.addData("Status", "Test Complete");
        telemetry.addData("Total Runtime", "%.1f sec", runtime.seconds());
        telemetry.update();

        // Clean shutdown
        testBench.disableMotor();
        testBench.homeServo();
    }

    /**
     * Run the complete test sequence
     */
    private void runTestSequence() {
        telemetry.addData("Status", "Starting Test Sequence");
        telemetry.update();

        // Test 1: Servo Positioning
        testServoPositioning();

        // Test 2: Motor Velocity Control
        testMotorVelocityControl();

        // Test 3: Motor Position Control
        testMotorPositionControl();

        telemetry.addData("Status", "All Tests Complete");
        telemetry.update();
    }

    /**
     * Test servo positioning functionality
     */
    private void testServoPositioning() {
        telemetry.addData("Test", "Servo Positioning");
        telemetry.update();

        // Home position
        testBench.homeServo();
        sleep((long) SERVO_TEST_DELAY_MS);
        telemetry.addData("Servo", "Home: %.2f", testBench.getServoPosition());
        telemetry.update();

        // Max position
        testBench.setServoPosition(1.0);
        sleep((long) SERVO_TEST_DELAY_MS);
        telemetry.addData("Servo", "Max: %.2f", testBench.getServoPosition());
        telemetry.update();

        // Min position
        testBench.setServoPosition(0.0);
        sleep((long) SERVO_TEST_DELAY_MS);
        telemetry.addData("Servo", "Min: %.2f", testBench.getServoPosition());
        telemetry.update();

        // Back to home
        testBench.homeServo();
        sleep((long) SERVO_TEST_DELAY_MS);
        telemetry.addData("Servo", "Home: %.2f", testBench.getServoPosition());
        telemetry.update();
    }

    /**
     * Test motor velocity control with PID
     */
    private void testMotorVelocityControl() {
        telemetry.addData("Test", "Motor Velocity Control");
        telemetry.update();

        // Test different velocities
        double[] testVelocities = {VELOCITY_TEST_1, VELOCITY_TEST_2, VELOCITY_TEST_3};

        for (double velocity : testVelocities) {
            telemetry.addData("Testing Velocity", "%.1f tps", velocity);
            telemetry.update();

            // Set velocity and enable motor
            testBench.setMotorVelocity(velocity);
            testBench.enableMotor();

            // Wait and monitor
            testTimer.reset();
            while (testTimer.milliseconds() < MOTOR_TEST_DELAY_MS && opModeIsActive()) {
                telemetry.addData("Current Vel", "%.1f / %.1f tps", 
                    testBench.getMotorVelocity(), velocity);
                telemetry.addData("At Target", testBench.isMotorAtTargetVelocity() ? "YES" : "NO");
                telemetry.addData("Position", "%d ticks", testBench.getMotorPosition());
                telemetry.update();
                sleep(50);
            }

            // Disable motor
            testBench.disableMotor();
            sleep(500);
        }
    }

    /**
     * Test motor position control
     */
    private void testMotorPositionControl() {
        telemetry.addData("Test", "Motor Position Control");
        telemetry.update();

        // Reset encoder
        testBench.disableMotor();
        sleep(100);

        // Test position control (this would require implementing position control in the subsystem)
        telemetry.addData("Position Test", "Not implemented in current subsystem");
        telemetry.addData("Note", "Use velocity control for precise motor control");
        telemetry.update();
        sleep(2000);
    }

    /**
     * Get test results summary
     */
    private String getTestResults() {
        return String.format(
            "Servo: %.2f | Motor: %.1f tps | Pos: %d | Runtime: %.1fs",
            testBench.getServoPosition(),
            testBench.getMotorVelocity(),
            testBench.getMotorPosition(),
            runtime.seconds()
        );
    }
}