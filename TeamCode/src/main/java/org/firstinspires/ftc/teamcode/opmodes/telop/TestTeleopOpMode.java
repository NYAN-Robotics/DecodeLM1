package org.firstinspires.ftc.teamcode.opmodes.telop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.utilities.subsystems.TestBenchSubsystem;

/**
 * Test TeleOp OpMode - Control PID motor and servo for testing
 *
 * Controls:
 * - A: Toggle motor on/off
 * - X: Increment servo position
 * - B: Decrement servo position
 * - Y: Home servo to center position
 * - Right Trigger: Increase motor velocity
 * - Left Trigger: Decrease motor velocity
 * - Back: Emergency stop all
 */
@TeleOp(name="Test Bench Control", group="Test")
public class TestTeleopOpMode extends OpMode {

    // Subsystems
    private TestBenchSubsystem testBench;

    // Timing
    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime buttonCooldown = new ElapsedTime();

    // Button state tracking
    private boolean lastA = false;
    private boolean lastX = false;
    private boolean lastB = false;
    private boolean lastY = false;
    private boolean lastBack = false;

    // Control variables
    private double customVelocity = 200.0; // Custom velocity control
    private static final double VELOCITY_INCREMENT = 50.0;
    private static final double BUTTON_COOLDOWN_MS = 200;

    @Override
    public void init() {
        // Initialize subsystem
        testBench = new TestBenchSubsystem();
        testBench.init(hardwareMap);

        // Reset runtime
        runtime.reset();
        buttonCooldown.reset();

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Controls", "A=Motor Toggle, X/B=Servo, Y=Home, Triggers=Velocity");
        telemetry.update();
    }

    @Override
    public void start() {
        runtime.reset();
        telemetry.addData("Status", "Started - Ready for Control");
        telemetry.update();
    }

    @Override
    public void loop() {
        handleButtonInputs();
        handleAnalogInputs();
        updateTelemetry();
    }

    /**
     * Handle digital button inputs with debouncing
     */
    private void handleButtonInputs() {
        boolean cooldownExpired = buttonCooldown.milliseconds() > BUTTON_COOLDOWN_MS;

        // Enable motor with A button
        if (gamepad1.a && !lastA && cooldownExpired) {
            testBench.enableMotor();
            buttonCooldown.reset();
        }

        // Servo control with X and B buttons
        if (gamepad1.x && !lastX && cooldownExpired) {
            testBench.incrementServoPosition();
            buttonCooldown.reset();
        }

        if (gamepad1.b && !lastB && cooldownExpired) {
            testBench.decrementServoPosition();
            buttonCooldown.reset();
        }

        // Disable motor
        if (gamepad1.y && !lastY && cooldownExpired) {
//            testBench.homeServo();
            testBench.disableMotor();
//            buttonCooldown.reset();
        }

        // Emergency stop with Back button
        if (gamepad1.back && !lastBack && cooldownExpired) {
            testBench.disableMotor();
            testBench.homeServo();
            customVelocity = 200.0;
            buttonCooldown.reset();
        }

        // Update last button states
        lastA = gamepad1.a;
        lastX = gamepad1.x;
        lastB = gamepad1.b;
        lastY = gamepad1.y;
        lastBack = gamepad1.back;
    }

    /**
     * Handle analog trigger inputs for velocity control
     */
    private void handleAnalogInputs() {
        // Velocity control with triggers
        if (gamepad1.right_trigger > 0.1) {
            customVelocity += VELOCITY_INCREMENT * gamepad1.right_trigger * 0.02; // Scale by loop time
            customVelocity = Math.min(customVelocity, 1000.0); // Max velocity limit

            if (testBench.isMotorEnabled()) {
                testBench.setMotorVelocity(customVelocity);
            }
        }

        if (gamepad1.left_trigger > 0.1) {
            customVelocity -= VELOCITY_INCREMENT * gamepad1.left_trigger * 0.02; // Scale by loop time
            customVelocity = Math.max(customVelocity, 0.0); // Min velocity limit

            if (testBench.isMotorEnabled()) {
                testBench.setMotorVelocity(customVelocity);
            }
        }
    }

    /**
     * Update telemetry display
     */
    private void updateTelemetry() {
        // Runtime and loop frequency
        telemetry.addData("Runtime", "%.1f sec", runtime.seconds());
        telemetry.addData("Loop Time", "%.1f ms", runtime.milliseconds() % 20);

        // Control status
        telemetry.addData("Motor Status", testBench.isMotorEnabled() ? "ENABLED" : "DISABLED");
        telemetry.addData("Motor Velocity", "%.1f / %.1f tps",
                testBench.getMotorVelocity(),
                testBench.getTargetVelocity());
        telemetry.addData("Custom Velocity", "%.1f tps", customVelocity);
        telemetry.addData("Motor Position", "%d ticks", testBench.getMotorPosition());
        telemetry.addData("At Target Vel", testBench.isMotorAtTargetVelocity() ? "YES" : "NO");

        // Servo status
        telemetry.addData("Servo Position", "%.2f", testBench.getServoPosition());

        // Quick status line
        telemetry.addData("Quick Status", testBench.getTelemetryData());

        // Control hints
        telemetry.addData("", "--- CONTROLS ---");
        telemetry.addData("A", "Toggle Motor");
        telemetry.addData("X/B", "Servo +/-");
        telemetry.addData("Y", "Home Servo");
        telemetry.addData("RT/LT", "Velocity +/-");
        telemetry.addData("Back", "Emergency Stop");

        telemetry.update();
    }

    @Override
    public void stop() {
        // Ensure everything stops safely
        testBench.disableMotor();
        testBench.homeServo();

        telemetry.addData("Status", "Stopped");
        telemetry.update();
    }
}