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
import org.firstinspires.ftc.teamcode.utilities.robot.RobotEx;
import org.firstinspires.ftc.teamcode.utilities.robot.command.framework.OneTimeCommand;
import org.firstinspires.ftc.teamcode.utilities.robot.command.framework.SequentialCommand;
import org.firstinspires.ftc.teamcode.utilities.robot.command.framework.YieldCommand;
import org.mercurialftc.mercurialftc.util.hardware.cachinghardwaredevice.CachingDcMotorEX;
import org.mercurialftc.mercurialftc.util.hardware.cachinghardwaredevice.CachingServo;

@Config
public class Outtake implements Subsystem {

    public enum OuttakeSlidesStates {
        DEFAULT(0),
        SAMPLES(2000),
        HANG(2100),
        SPECIMENS(700),
        SPECIMENS_DROP(0),
        SPECIMEN_PICKUP (400),
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
        DEFAULT(0.44),
        BACK_PICKUP(0.25),
        AUTO_DEFAULT(0.37),
        HANG_INITIAL(0.60),
        HANG_FINAL(0.7),
        EXTENDED(0.86),
        SPECIMEN_DROP(1),
        SPECIMEN_PICKUP(0.33);

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
        DEFAULT(0.53),
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
        FULL_DEFAULT(0),
        DEFAULT(0.55),
        CLOSED(0.76);

        public double position;

        // Constructor
        OuttakeClawStates(double position) {
            this.position = position;
        }

        public void setPosition(double position) {
            this.position = position;
        }

    }

    public enum OuttakePivotStates {
        DEFAULT(0.83),
        TRANSFER_POSITION(0.6),
        SAMPLE_DROP(1),
        SPECIMEN_PICKUP(0.2),
        DOWN(0);

        public double position;

        // Constructor
        OuttakePivotStates(double position) {
            this.position = position;
        }

        public void setPosition(double position) {
            this.position = position;
        }

    }


    public static double defaultPivotPosition = OuttakePivotStates.DEFAULT.position;
    public static double extendedPivotPosition = OuttakePivotStates.DOWN.position;

    public static double defaultOuttakeRotationPosition = OuttakeServoState.DEFAULT.position;
    public static double rotatedOuttakeRotationPosition = OuttakeServoState.EXTENDED.position;

    DcMotorEx leftLiftMotor;
    DcMotorEx rightLiftMotor;

    DcMotorEx slidesEncoderMotor;

    Servo leftOuttakeServo;
    Servo rightOuttakeServo;

    Servo rotationOuttakeServo;
    Servo clawServo;

    Servo clawPivot;

    DcMotorEx intakeMotor;

    DigitalChannel magneticLimitSwitch;

    AnalogInput outtakeAnalog;

    OuttakeServoState currentOuttakeServoState = OuttakeServoState.DEFAULT;
    OuttakeRotationStates currentRotationState = OuttakeRotationStates.DEFAULT;
    OuttakeClawStates currentClawState = OuttakeClawStates.DEFAULT;
    OuttakeSlidesStates currentSlideState = OuttakeSlidesStates.DEFAULT;
    OuttakeSlidesStates previousSlideState = OuttakeSlidesStates.DEFAULT;
    OuttakePivotStates currentPivotState = OuttakePivotStates.DEFAULT;

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

    RobotEx robot;

    @Override
    public void onInit(HardwareMap hardwareMap, Telemetry telemetry) {
        leftLiftMotor = new CachingDcMotorEX(hardwareMap.get(DcMotorEx.class, "leftLiftMotor"), 1e-5);
        rightLiftMotor = new CachingDcMotorEX(hardwareMap.get(DcMotorEx.class, "rightLiftMotor"), 1e-5);

        slidesEncoderMotor = hardwareMap.get(DcMotorEx.class, "rightBackMotor");

        leftOuttakeServo = new CachingServo(hardwareMap.get(Servo.class, "leftOuttakeServo"), 1e-5);
        rightOuttakeServo = new CachingServo(hardwareMap.get(Servo.class, "rightOuttakeServo"), 1e-5);

        rotationOuttakeServo = new CachingServo(hardwareMap.get(Servo.class, "clawRotation"), 1e-5);

        clawServo = new CachingServo(hardwareMap.get(Servo.class, "claw"), 1e-5);
        clawPivot = new CachingServo(hardwareMap.get(Servo.class, "clawPivot"), 1e-5);

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
        robot = RobotEx.getInstance();
    }

    @Override
    public void onCyclePassed() {

        OuttakePivotStates.DEFAULT.position = defaultPivotPosition;
        OuttakePivotStates.DOWN.position = extendedPivotPosition;

        OuttakeServoState.DEFAULT.position = defaultOuttakeRotationPosition;
        OuttakeServoState.EXTENDED.position = rotatedOuttakeRotationPosition;

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
            } else if (currentSlideState != OuttakeSlidesStates.SPECIMEN_PICKUP && currentSlideState != OuttakeSlidesStates.SPECIMENS_DROP){
                setCurrentOuttakeState(OuttakeServoState.EXTENDED);
                setCurrentRotationState(OuttakeRotationStates.DEFAULT);
            }

            if (currentSlideState == OuttakeSlidesStates.SPECIMENS_DROP || currentSlideState == OuttakeSlidesStates.SAMPLES) {
                setCurrentRotationState(OuttakeRotationStates.ROTATED);
                setCurrentPivotState(OuttakePivotStates.SAMPLE_DROP);
            }

            outtakeReset = true;
        }

        if (previousSlideState == OuttakeSlidesStates.DEFAULT && currentSlideState == OuttakeSlidesStates.SAMPLES && profile.timer.seconds() < 0.5) {
            setCurrentPivotState(OuttakePivotStates.TRANSFER_POSITION);
        }

        leftLiftMotor.setPower(liftPower);
        rightLiftMotor.setPower(liftPower);

        leftOuttakeServo.setPosition(currentOuttakeServoState.position);
        rightOuttakeServo.setPosition(currentOuttakeServoState.position);

        rotationOuttakeServo.setPosition(currentRotationState.position);
        clawServo.setPosition(currentClawState.position);

        clawPivot.setPosition(currentPivotState.position);

        telemetry.addData("Outtake Servo State: ", currentOuttakeServoState);
        telemetry.addData("Outtake Position: ", currentOuttakeServoState.position);

        telemetry.addData("Rotation State: ", currentRotationState);
        telemetry.addData("Pivot State: ", currentPivotState);
        telemetry.addData("Claw State: ", currentClawState);
        telemetry.addData("Position: ", getCurrentSensorPosition());
        telemetry.addData("Target Position: ", profile.feedforwardProfile.getPositionFromTime(slidesTimer.seconds()));
        telemetry.addData("Magnetic Switch: ", currentSwitchState);
        telemetry.addData("Analog Position Outtake: ", outtakeAnalog.getVoltage());

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

    public void setCurrentPivotState(OuttakePivotStates newState) {
        currentPivotState = newState;
    }

    public void setCurrentClawState(OuttakeClawStates newState) {

        if (newState == OuttakeClawStates.CLOSED && currentSlideState == OuttakeSlidesStates.SPECIMEN_PICKUP) {

            robot.commandScheduler.scheduleCommand(
                    new SequentialCommand(
                            new YieldCommand(1000),
                            new OneTimeCommand(() -> setSlidesState(OuttakeSlidesStates.SPECIMENS)),
                            new YieldCommand(2000, this::atTargetPosition),
                            new OneTimeCommand(() -> setCurrentOuttakeState(OuttakeServoState.SPECIMEN_DROP)),
                            new OneTimeCommand(() -> setCurrentPivotState(OuttakePivotStates.TRANSFER_POSITION)),
                            new OneTimeCommand(() -> setCurrentRotationState(OuttakeRotationStates.ROTATED)),
                            new YieldCommand(2000),
                            new OneTimeCommand(() -> setCurrentOuttakeState(OuttakeServoState.SPECIMEN_DROP))
                    )
            );
        }

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
        setCurrentPivotState(OuttakePivotStates.DEFAULT);
    }
}
