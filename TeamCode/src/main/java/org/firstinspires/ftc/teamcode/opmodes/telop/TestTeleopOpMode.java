package org.firstinspires.ftc.teamcode.opmodes.telop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.utilities.subsystems.DrivetrainSubsystem;
import org.firstinspires.ftc.teamcode.utilities.subsystems.TestBenchSubsystem;
import org.firstinspires.ftc.teamcode.utilities.config.core.robotConstants;

/**
 * Test TeleOp OpMode - Control two-motor drivetrain with PID and RPM control
 *
 * Controls:
 * - Left Joystick Y: Control left motor speed (-1.0 to 1.0)
 * - Right Joystick Y: Control right motor speed (-1.0 to 1.0)
 * - A: Toggle motors on/off
 * - X: Increment RPM
 * - B: Decrement RPM
 * - Y: Reset to default RPM
 * - Right Trigger: Increase RPM
 * - Left Trigger: Decrease RPM
 * - D-Pad Up: Set high RPM (2000)
 * - D-Pad Down: Set low RPM (500)
 * - D-Pad Left: Set left motor only
 * - D-Pad Right: Set right motor only
 * - Back: Emergency stop all
 */
@TeleOp(name="Two Motor PID Control", group="Test")
public class TestTeleopOpMode extends OpMode {

    // Subsystems
    private DrivetrainSubsystem drivetrain;
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
    private boolean lastDpadUp = false;
    private boolean lastDpadDown = false;
    private boolean lastDpadLeft = false;
    private boolean lastDpadRight = false;

    // Control variables
    private double customRPM = robotConstants.DEFAULT_TARGET_RPM; // Custom RPM control
    private static final double RPM_INCREMENT = robotConstants.RPM_INCREMENT;
    private static final double BUTTON_COOLDOWN_MS = 200;
    
    // Joystick control variables
    private boolean joystickControlEnabled = true; // Enable/disable joystick control
    private double maxJoystickRPM = 2000.0; // Maximum RPM when joystick is fully up

    @Override
    public void init() {
        // Initialize subsystems
        drivetrain = new DrivetrainSubsystem();
        drivetrain.init(hardwareMap);
        
        testBench = new TestBenchSubsystem();
        testBench.init(hardwareMap);

        // Reset runtime
        runtime.reset();
        buttonCooldown.reset();

        telemetry.addData("Status", "Initialized - Two Motor PID Control");
        telemetry.addData("Controls", "Left/Right Joystick Y = Motor Speed");
        telemetry.addData("Buttons", "A=Toggle Motors, X/B=RPM +/-, Y=Reset RPM");
        telemetry.addData("D-Pad", "Up=High RPM, Down=Low RPM, L/R=Single Motor");
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
        handleJoystickInputs();
        handleButtonInputs();
        handleAnalogInputs();
        updateTelemetry();
    }

    /**
     * Handle joystick inputs for motor control
     */
    private void handleJoystickInputs() {
        if (joystickControlEnabled) {
            // Get joystick Y values (inverted because joystick up is negative)
            double leftJoystickY = -gamepad1.left_stick_y; // Invert so up is positive
            double rightJoystickY = -gamepad1.right_stick_y; // Invert so up is positive
            
            // Apply deadzone to prevent drift
            double deadzone = 0.1;
            if (Math.abs(leftJoystickY) < deadzone) leftJoystickY = 0.0;
            if (Math.abs(rightJoystickY) < deadzone) rightJoystickY = 0.0;
            
            // Convert joystick values to RPM
            // Joystick value -1.0 to 1.0 maps to RPM 0 to maxJoystickRPM
            double leftMotorRPM = leftJoystickY * maxJoystickRPM;
            double rightMotorRPM = rightJoystickY * maxJoystickRPM;
            
            // Clamp RPM to safe bounds
            leftMotorRPM = Math.max(0.0, Math.min(maxJoystickRPM, leftMotorRPM));
            rightMotorRPM = Math.max(0.0, Math.min(maxJoystickRPM, rightMotorRPM));
            
            // Set motor RPMs
            drivetrain.setLeftMotorRPM(leftMotorRPM);
            drivetrain.setRightMotorRPM(rightMotorRPM);
            
            // Update custom RPM for button controls
            customRPM = Math.max(leftMotorRPM, rightMotorRPM);
        }
    }

    /**
     * Handle digital button inputs with debouncing
     */
    private void handleButtonInputs() {
        boolean cooldownExpired = buttonCooldown.milliseconds() > BUTTON_COOLDOWN_MS;

        // Toggle motors with A button
        if (gamepad1.a && !lastA && cooldownExpired) {
            drivetrain.toggleMotors();
            buttonCooldown.reset();
        }

        // RPM control with X and B buttons
        if (gamepad1.x && !lastX && cooldownExpired) {
            drivetrain.incrementRPM();
            customRPM = drivetrain.getCurrentTargetRPM();
            buttonCooldown.reset();
        }

        if (gamepad1.b && !lastB && cooldownExpired) {
            drivetrain.decrementRPM();
            customRPM = drivetrain.getCurrentTargetRPM();
            buttonCooldown.reset();
        }

        // Reset to default RPM with Y button
        if (gamepad1.y && !lastY && cooldownExpired) {
            drivetrain.setBothMotorsRPM(robotConstants.DEFAULT_TARGET_RPM);
            customRPM = robotConstants.DEFAULT_TARGET_RPM;
            buttonCooldown.reset();
        }

        // D-Pad controls
        if (gamepad1.dpad_up && !lastDpadUp && cooldownExpired) {
            drivetrain.setBothMotorsRPM(2000.0); // High RPM
            customRPM = 2000.0;
            buttonCooldown.reset();
        }

        if (gamepad1.dpad_down && !lastDpadDown && cooldownExpired) {
            drivetrain.setBothMotorsRPM(500.0); // Low RPM
            customRPM = 500.0;
            buttonCooldown.reset();
        }

        if (gamepad1.dpad_left && !lastDpadLeft && cooldownExpired) {
            // Set only left motor
            drivetrain.setLeftMotorRPM(customRPM);
            drivetrain.setRightMotorRPM(0.0);
            buttonCooldown.reset();
        }

        if (gamepad1.dpad_right && !lastDpadRight && cooldownExpired) {
            // Set only right motor
            drivetrain.setLeftMotorRPM(0.0);
            drivetrain.setRightMotorRPM(customRPM);
            buttonCooldown.reset();
        }

        // Emergency stop with Back button
        if (gamepad1.back && !lastBack && cooldownExpired) {
            drivetrain.disableMotors();
            testBench.disableMotor();
            testBench.homeServo();
            customRPM = robotConstants.DEFAULT_TARGET_RPM;
            buttonCooldown.reset();
        }

        // Update last button states
        lastA = gamepad1.a;
        lastX = gamepad1.x;
        lastB = gamepad1.b;
        lastY = gamepad1.y;
        lastBack = gamepad1.back;
        lastDpadUp = gamepad1.dpad_up;
        lastDpadDown = gamepad1.dpad_down;
        lastDpadLeft = gamepad1.dpad_left;
        lastDpadRight = gamepad1.dpad_right;
    }

    /**
     * Handle analog trigger inputs for RPM control
     */
    private void handleAnalogInputs() {
        // RPM control with triggers
        if (gamepad1.right_trigger > 0.1) {
            customRPM += RPM_INCREMENT * gamepad1.right_trigger * 0.02; // Scale by loop time
            customRPM = Math.min(customRPM, robotConstants.MAX_RPM); // Max RPM limit

            if (drivetrain.areMotorsEnabled()) {
                drivetrain.setBothMotorsRPM(customRPM);
            }
        }

        if (gamepad1.left_trigger > 0.1) {
            customRPM -= RPM_INCREMENT * gamepad1.left_trigger * 0.02; // Scale by loop time
            customRPM = Math.max(customRPM, robotConstants.MIN_RPM); // Min RPM limit

            if (drivetrain.areMotorsEnabled()) {
                drivetrain.setBothMotorsRPM(customRPM);
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

        // Drivetrain status
        telemetry.addData("Motors Status", drivetrain.areMotorsEnabled() ? "ENABLED" : "DISABLED");
        telemetry.addData("Joystick Control", joystickControlEnabled ? "ON" : "OFF");
        telemetry.addData("Max Joystick RPM", "%.0f RPM", maxJoystickRPM);
        
        // Joystick values
        telemetry.addData("Left Joystick", "%.2f", -gamepad1.left_stick_y);
        telemetry.addData("Right Joystick", "%.2f", -gamepad1.right_stick_y);
        
        // Left motor details
        telemetry.addData("Left Motor", "%.0f/%.0f RPM", 
                drivetrain.getLeftMotorRPM(), 
                drivetrain.getLeftTargetRPM());
        telemetry.addData("Left At Target", drivetrain.isLeftMotorAtTargetRPM() ? "YES" : "NO");
        
        // Right motor details
        telemetry.addData("Right Motor", "%.0f/%.0f RPM", 
                drivetrain.getRightMotorRPM(), 
                drivetrain.getRightTargetRPM());
        telemetry.addData("Right At Target", drivetrain.isRightMotorAtTargetRPM() ? "YES" : "NO");
        
        // Both motors at target
        telemetry.addData("Both At Target", drivetrain.areMotorsAtTargetRPM() ? "YES" : "NO");

        // Servo status (from test bench)
        telemetry.addData("Servo Position", "%.2f", testBench.getServoPosition());

        // Detailed status line
        telemetry.addData("Drivetrain Status", drivetrain.getTelemetryData());

        // Control hints
        telemetry.addData("", "--- CONTROLS ---");
        telemetry.addData("Left Stick Y", "Left Motor Speed");
        telemetry.addData("Right Stick Y", "Right Motor Speed");
        telemetry.addData("A", "Toggle Motors");
        telemetry.addData("X/B", "RPM +/-");
        telemetry.addData("Y", "Reset RPM");
        telemetry.addData("RT/LT", "RPM +/-");
        telemetry.addData("D-Pad", "Up=High, Down=Low, L/R=Single Motor");
        telemetry.addData("Back", "Emergency Stop");

        telemetry.update();
    }

    @Override
    public void stop() {
        // Ensure everything stops safely
        drivetrain.disableMotors();
        testBench.disableMotor();
        testBench.homeServo();

        telemetry.addData("Status", "Stopped - All Motors Disabled");
        telemetry.update();
    }
}