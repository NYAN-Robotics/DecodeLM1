package org.firstinspires.ftc.teamcode.utilities.robot.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.utilities.controltheory.MotionProfiledMotion;
import org.firstinspires.ftc.teamcode.utilities.controltheory.feedback.GeneralPIDController;
import org.firstinspires.ftc.teamcode.utilities.controltheory.motionprofiler.MotionProfile;
import org.mercurialftc.mercurialftc.util.hardware.cachinghardwaredevice.CachingDcMotorEX;
import org.mercurialftc.mercurialftc.util.hardware.cachinghardwaredevice.CachingServo;

@Config
public class Outtake implements Subsystem {

    public enum OuttakeSlidesStates {
        DEFAULT(0),
        SAMPLES(2250),
        SPECIMENS(700),
        SPECIMENS_DROP(190),
        HOVER(350);

        public double position;

        // Constructor
        OuttakeSlidesStates(double position) {
            this.position = position;
        }

        public void setPosition(double position) {
            this.position = position;
        }

    }

    public enum OuttakeServoState {
        DEFAULT(0.625),
        BACK_PICKUP(0.95),
        AUTO_DEFAULT(0.37),
        EXTENDED(0.1);

        public double position;

        // Constructor
        OuttakeServoState(double position) {
            this.position = position;
        }

        public void setPosition(double position) {
            this.position = position;
        }

    }

    public enum OuttakeRotationStates {
        DEFAULT(0.5),
        ROTATED(0.18);

        public double position;

        // Constructor
        OuttakeRotationStates(double position) {
            this.position = position;
        }

        public void setPosition(double position) {
            this.position = position;
        }

    }

    public enum OuttakeClawStates {
        FULL_DEFAULT(1),
        DEFAULT(0.85),
        CLOSED(0.35);

        public double position;

        // Constructor
        OuttakeClawStates(double position) {
            this.position = position;
        }

        public void setPosition(double position) {
            this.position = position;
        }

    }

    DcMotorEx leftLiftMotor;
    DcMotorEx rightLiftMotor;

    DcMotorEx slidesEncoderMotor;

    Servo leftOuttakeServo;
    Servo rightOuttakeServo;

    Servo rotationOuttakeServo;
    Servo clawServo;

    DcMotorEx intakeMotor;

    DigitalChannel magneticLimitSwitch;

    AnalogInput outtakeAnalog;

    OuttakeServoState currentOuttakeServoState = OuttakeServoState.DEFAULT;
    OuttakeRotationStates currentRotationState = OuttakeRotationStates.DEFAULT;
    OuttakeClawStates currentClawState = OuttakeClawStates.DEFAULT;
    OuttakeSlidesStates currentSlideState = OuttakeSlidesStates.DEFAULT;
    OuttakeSlidesStates previousSlideState = OuttakeSlidesStates.DEFAULT;

    public static double kP = 0.0025;
    public static double kI = 0;
    public static double kD = 0.0001;
    public static double kF = 0.1;
    public static double kV = 0.0005;
    public static double kA = 0;
    public static double vMax = 5000;
    public static double aMax = 8000;

    private GeneralPIDController controller = new GeneralPIDController(kP, kI, kD, kF);
    public ElapsedTime slidesTimer = new ElapsedTime();

    private MotionProfiledMotion profile = new MotionProfiledMotion(
            new MotionProfile(0, 0, vMax, aMax),
            new GeneralPIDController(kP, kI, kD, kF)
    );

    double liftPower = 0;
    boolean currentSwitchState = false;
    boolean pushingDown = false;
    boolean outtakeReset = false;

    Telemetry telemetry;

    @Override
    public void onInit(HardwareMap hardwareMap, Telemetry telemetry) {
        leftLiftMotor = new CachingDcMotorEX(hardwareMap.get(DcMotorEx.class, "leftLiftMotor"), 1e-5);
        rightLiftMotor = new CachingDcMotorEX(hardwareMap.get(DcMotorEx.class, "rightLiftMotor"), 1e-5);

        slidesEncoderMotor = hardwareMap.get(DcMotorEx.class, "rightBackMotor");

        leftOuttakeServo = new CachingServo(hardwareMap.get(Servo.class, "leftOuttakeServo"), 1e-5);
        rightOuttakeServo = new CachingServo(hardwareMap.get(Servo.class, "rightOuttakeServo"), 1e-5);

        rotationOuttakeServo = new CachingServo(hardwareMap.get(Servo.class, "clawRotation"), 1e-5);

        clawServo = new CachingServo(hardwareMap.get(Servo.class, "claw"), 1e-5);

        intakeMotor = new CachingDcMotorEX(hardwareMap.get(DcMotorEx.class, "intakeMotor"), 1e-5);

        magneticLimitSwitch = hardwareMap.get(DigitalChannel.class, "slidesMagnetic");
        outtakeAnalog = hardwareMap.get(AnalogInput.class, "outtakeAnalog");

        leftOuttakeServo.setDirection(Servo.Direction.FORWARD); // dir
        rightOuttakeServo.setDirection(Servo.Direction.REVERSE); // Dir

        leftLiftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightLiftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        leftLiftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rightLiftMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        leftLiftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightLiftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        clawServo.setDirection(Servo.Direction.REVERSE);
        this.telemetry = telemetry;
    }

    @Override
    public void onOpmodeStarted() {

    }

    @Override
    public void onCyclePassed() {

        profile.setPIDCoefficients(kP, kI, kD, kF);
        profile.setProfileCoefficients(kV, kA, vMax, aMax);

        currentSwitchState = !magneticLimitSwitch.getState();

        if (atTargetPosition() && this.currentSlideState == OuttakeSlidesStates.DEFAULT) {
            liftPower /= 2;

            if (currentSwitchState && liftPower == 0 && profile.timer.seconds() - profile.feedforwardProfile.getDuration() < 0.5) {
                liftPower = -0.2;
                pushingDown = true;
            } else if (pushingDown) {
                pushingDown = false;
                leftLiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                leftLiftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                rightLiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rightLiftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            }
        }
        if (liftPower == 0) {
            liftPower = profile.getOutput(getCurrentSensorPosition());
        }

        if (currentSlideState != OuttakeSlidesStates.DEFAULT && profile.atTargetPosition() && !outtakeReset) {
            if (currentSlideState == OuttakeSlidesStates.HOVER) {
                setCurrentOuttakeState(OuttakeServoState.BACK_PICKUP);
            } else {
                setCurrentOuttakeState(OuttakeServoState.EXTENDED);
                setCurrentRotationState(OuttakeRotationStates.DEFAULT);
            }

            if (currentSlideState == OuttakeSlidesStates.SPECIMENS || currentSlideState == OuttakeSlidesStates.SPECIMENS_DROP || currentSlideState == OuttakeSlidesStates.SAMPLES) {
                setCurrentRotationState(OuttakeRotationStates.ROTATED);
            }

            outtakeReset = true;
        }

        leftLiftMotor.setPower(liftPower);
        rightLiftMotor.setPower(liftPower);

        // leftOuttakeServo.setPosition(currentOuttakeServoState.position);
        // rightOuttakeServo.setPosition(currentOuttakeServoState.position);

        // rotationOuttakeServo.setPosition(currentRotationState.position);
        // clawServo.setPosition(currentClawState.position);

        telemetry.addData("Outtake Servo State: ", currentOuttakeServoState);
        telemetry.addData("Outtake Position: ", currentOuttakeServoState.position);

        telemetry.addData("Rotation State: ", currentRotationState);
        telemetry.addData("Claw State: ", currentClawState);
        telemetry.addData("Position: ", getCurrentSensorPosition());
        telemetry.addData("Target Position: ", profile.feedforwardProfile.getPositionFromTime(slidesTimer.seconds()));
        telemetry.addData("Magnetic Switch: ", currentSwitchState);
        telemetry.addData("Analog Position Outtake: ", outtakeAnalog.getVoltage());

        telemetry.addData("leftmotor: ", leftLiftMotor);
        liftPower = 0;
    }

    public void setLiftPower(double liftPower) {
        this.liftPower = liftPower;
    }

    public void setSlidesState(OuttakeSlidesStates newState) {

        if (newState == currentSlideState) {
            return;
        }


        previousSlideState = currentSlideState;
        currentSlideState = newState;

        outtakeReset = false;
        regenerateProfile();
    }

    public void regenerateProfile() {
        slidesTimer.reset();

        profile.updateTargetPosition(currentSlideState.position, getCurrentSensorPosition());
    }

    public double getCurrentSensorPosition() {
        return -slidesEncoderMotor.getCurrentPosition();
    }

    public void setCurrentOuttakeState(OuttakeServoState newState) {
        currentOuttakeServoState = newState;
    }

    public void setCurrentRotationState(OuttakeRotationStates newState) {
        currentRotationState = newState;
    }

    public void setCurrentClawState(OuttakeClawStates newState) {
        currentClawState = newState;
    }

    public boolean atTargetPosition() {
        return profile.atTargetPosition();
    }

    public OuttakeSlidesStates getSlidesState() {
        return currentSlideState;
    }

    public OuttakeClawStates getClawState() {
        return currentClawState;
    }

    public void reset() {
        setSlidesState(Outtake.OuttakeSlidesStates.DEFAULT);
        setCurrentOuttakeState(Outtake.OuttakeServoState.DEFAULT);
        setCurrentClawState(Outtake.OuttakeClawStates.DEFAULT);
        setCurrentRotationState(Outtake.OuttakeRotationStates.DEFAULT);
    }
}
