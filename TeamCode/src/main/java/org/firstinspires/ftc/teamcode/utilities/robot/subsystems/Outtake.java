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
import org.firstinspires.ftc.teamcode.utilities.robot.Globals;
import org.firstinspires.ftc.teamcode.utilities.robot.RobotEx;
import org.firstinspires.ftc.teamcode.utilities.robot.command.framework.commandtypes.OneTimeCommand;
import org.firstinspires.ftc.teamcode.utilities.robot.command.framework.commandtypes.SequentialCommandGroup;
import org.firstinspires.ftc.teamcode.utilities.robot.command.framework.commandtypes.YieldCommand;
import org.mercurialftc.mercurialftc.util.hardware.cachinghardwaredevice.CachingDcMotorEX;
import org.mercurialftc.mercurialftc.util.hardware.cachinghardwaredevice.CachingServo;

@Config
public class Outtake implements Subsystem {

    public enum OuttakeSlidesStates {
        DEFAULT(0),
        SAMPLES(1740),
        SAMPLES_LOW(650),
        HANG(1730),
        HANG_FINAL(800),
        SPECIMENS(1150),
        SPECIMEN_TRANSFER(1300),
        SPECIMENS_DROP(950),
        SPECIMEN_INITIAL_PICKUP(400),
        SPECIMEN_PICKUP(50),
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
        DEFAULT(0.505), // DEFAULT
        BACK_PICKUP(DEFAULT.position - 0.23),
        AUTO_DEFAULT(DEFAULT.position - 0.10),
        HANG_INITIAL(DEFAULT.position + 0.11),
        HANG_FINAL(DEFAULT.position + 0.21),
        EXTENDED(DEFAULT.position + 0.41), // TWEAK
        EXTENDED_INITIAL(DEFAULT.position + 0.25),
        SPECIMEN_INITIAL(DEFAULT.position + 0.10),
        AUTO_PARK(DEFAULT.position + 0.43),
        SPECIMEN_PICKUP(DEFAULT.position - 0.24),
        SPECIMEN_PICKUP_2(DEFAULT.position - 0.24);

        public double position;


        // Constructor
        OuttakeServoState(double position) {
            this.position = position;
        }

        public void setPosition(double position) {
            this.position = position;
        }
    }
    /*
    public enum OuttakeServoState {
        DEFAULT(0.44+0.02),
        BACK_PICKUP(0.25+0.02),
        AUTO_DEFAULT(0.37+0.02),
        HANG_INITIAL(0.60+0.02),
        HANG_FINAL(0.7+0.02),
        EXTENDED(0.86+0.02+0.01),
        SPECIMEN_INITIAL(0.59),
        SPECIMEN_DROP_FINAL(0.62),
        UP(0.7),
        AUTO_PARK(0.9),
        SPECIMEN_PICKUP(0.31+0.02);

        public double position;

        // Constructor
        OuttakeServoState(double position) {
            this.position = position;
        }

        public void setPosition(double position) {
            this.position = position;
        }

    }

     */

    public enum OuttakeRotationStates {
        DEFAULT(0.50),
        SPECIMEN_ROTATED(0.22),
        ROTATED(0.78);

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
        DEFAULT(0.48 + 0.03),
        SPECIMEN_PICKUP(DEFAULT.position + 0.05 - 0.03),
        CLOSED(DEFAULT.position - 0.06 - 0.03),
        LESS_CLOSED(DEFAULT.position - 0.082);
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
        DEFAULT(0.67), // FIGURE OUT
        TRANSFER_POSITION(DEFAULT.position - .1),
        SPECIMEN_INITIAL(DEFAULT.position + 0.1),
        SPECIMEN_DROP(DEFAULT.position + 0),
        SAMPLE_DROP(DEFAULT.position + 0.2), // FIGURE OUT
        SPECIMEN_PICKUP(DEFAULT.position - 0.57),
        HORIZONTAL_SAMPLE_DROP(SAMPLE_DROP.position - 0.1),
        DOWN(DEFAULT.position - 0.05); // FIGURE OUT

        public double position;

        // Constructor
        OuttakePivotStates(double position) {
            this.position = position;
        }

        public void setPosition(double position) {
            this.position = position;
        }
    }


    public static double defaultSpecimenPosition = OuttakeSlidesStates.SPECIMENS.position;

    public static double defaultServoSpecimenPosition = OuttakeServoState.SPECIMEN_INITIAL.position;

    public static double defaultPivotPosition = OuttakePivotStates.DEFAULT.position;
    public static double extendedPivotPosition = OuttakePivotStates.DOWN.position;

    public static double defaultOuttakeRotationPosition = OuttakeServoState.DEFAULT.position;
    public static double rotatedOuttakeRotationPosition = OuttakeServoState.EXTENDED.position;

    public static double defaultSpecimenInitialPosition = OuttakePivotStates.SPECIMEN_INITIAL.position;

    public static double defaultOuttakeEffectorRotationPosition = OuttakeRotationStates.DEFAULT.position;

    public static double defaultOuttakeClawStates = OuttakeClawStates.DEFAULT.position;

    public static double specimenPivotClawPosition = OuttakePivotStates.SPECIMEN_DROP.position;

    DcMotorEx leftLiftMotor;
    DcMotorEx rightLiftMotor;
    DcMotorEx centerLiftMotor;

    DcMotorEx slidesEncoderMotor;

    Servo leftOuttakeServo;
    Servo rightOuttakeServo;

    Servo rotationOuttakeServo;
    public Servo clawServo;

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

    public static double kP = 0.0045;
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
    double positionDrift = 0;
    boolean currentSwitchState = false;
    boolean pushingDown = false;
    boolean outtakeReset = false;

    boolean sampleServoRotatedRequested = false;

    boolean waitingForIntake = false;

    Telemetry telemetry;

    RobotEx robot;

    @Override
    public void onInit(HardwareMap hardwareMap, Telemetry telemetry) {
        leftLiftMotor = new CachingDcMotorEX(hardwareMap.get(DcMotorEx.class, "leftLiftMotor"), 1e-5);
        rightLiftMotor = new CachingDcMotorEX(hardwareMap.get(DcMotorEx.class, "rightLiftMotor"), 1e-5);
        centerLiftMotor = new CachingDcMotorEX(hardwareMap.get(DcMotorEx.class, "centerLiftMotor"), 1e-5);

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
        rightOuttakeServo.setDirection(Servo.Direction.REVERSE); // dir

        leftLiftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightLiftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        centerLiftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        leftLiftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        rightLiftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        centerLiftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        leftLiftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightLiftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        centerLiftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        slidesEncoderMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slidesEncoderMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

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

        OuttakeServoState.SPECIMEN_INITIAL.position = defaultServoSpecimenPosition;

        OuttakeSlidesStates.SPECIMENS.position = defaultSpecimenPosition;

        OuttakePivotStates.SPECIMEN_INITIAL.position = defaultSpecimenInitialPosition;

        OuttakeRotationStates.DEFAULT.position = defaultOuttakeEffectorRotationPosition;

        OuttakeClawStates.DEFAULT.position = defaultOuttakeClawStates;

        OuttakePivotStates.SPECIMEN_DROP.position = specimenPivotClawPosition;

        profile.setPIDCoefficients(kP, kI, kD, kF);
        profile.setProfileCoefficients(kV, kA, vMax, aMax);

        currentSwitchState = !magneticLimitSwitch.getState();


        if (atTargetPosition() && this.currentSlideState == OuttakeSlidesStates.DEFAULT) {
            liftPower /= 2;

            if (currentSwitchState && liftPower <= 0.005 && profile.timer.seconds() - profile.feedforwardProfile.getDuration() < 0.5) {
                liftPower = -0.2;
                pushingDown = true;
            } else if (pushingDown) {
                pushingDown = false;
                positionDrift += getCurrentSensorPosition();
                System.out.println("Drift: " + positionDrift);
            }
        }

        if (liftPower == 0) {
            liftPower = profile.getOutput(getCurrentSensorPosition());
        }

        if (currentSlideState != OuttakeSlidesStates.DEFAULT && profile.atTargetPosition() && !outtakeReset) {
            if (currentSlideState == OuttakeSlidesStates.HOVER) {
                setCurrentOuttakeState(OuttakeServoState.BACK_PICKUP);
            } else if (currentSlideState == OuttakeSlidesStates.HANG) {
                setCurrentOuttakeState(OuttakeServoState.HANG_INITIAL);
            }

            if (currentSlideState == OuttakeSlidesStates.SAMPLES || currentSlideState == OuttakeSlidesStates.SAMPLES_LOW) {
                setCurrentOuttakeState(OuttakeServoState.EXTENDED);
                setCurrentRotationState(OuttakeRotationStates.ROTATED);
                setCurrentPivotState(OuttakePivotStates.SAMPLE_DROP);
            }

            outtakeReset = true;
        }
        if (previousSlideState == OuttakeSlidesStates.DEFAULT && currentSlideState == OuttakeSlidesStates.SAMPLES_LOW &&  profile.timer.seconds() > 0.15 && profile.timer.seconds() < 0.5 && !sampleServoRotatedRequested) {
            setCurrentPivotState(OuttakePivotStates.TRANSFER_POSITION);
            robot.theIntake.setIntakeState(Intake.IntakeState.TRANSFER);


        }

        if (previousSlideState == OuttakeSlidesStates.DEFAULT && currentSlideState == OuttakeSlidesStates.SAMPLES &&  profile.timer.seconds() > 0.15 && profile.timer.seconds() < 0.5 && !sampleServoRotatedRequested) {
            setCurrentPivotState(OuttakePivotStates.TRANSFER_POSITION);

            robot.theIntake.setIntakeState(Intake.IntakeState.TRANSFER);

            /*
            robot.theCommandScheduler.scheduleCommand(
                    new SequentialCommandGroup(
                            new YieldCommand(500),
                            new OneTimeCommand(() -> robot.theIntake.setIntakeState(Intake.IntakeState.DEFAULT))
                    )
            );


             */

            if (true || !Globals.inTeleop) {
                sampleServoRotatedRequested = true;

                robot.theCommandScheduler.scheduleCommand(
                        new SequentialCommandGroup(
                                new YieldCommand(200),
                                new OneTimeCommand(() ->
                                {
                                    if (currentSlideState == OuttakeSlidesStates.SAMPLES) {

                                        setCurrentOuttakeState(OuttakeServoState.EXTENDED);
                                        setCurrentRotationState(OuttakeRotationStates.ROTATED);
                                    }
                                    sampleServoRotatedRequested = false;
                                }),
                                new YieldCommand(2000, this::atTargetPosition),
                                new OneTimeCommand(() -> { if (currentSlideState == OuttakeSlidesStates.SAMPLES) { setCurrentPivotState(OuttakePivotStates.SAMPLE_DROP); } })

                        )
                );
            }
        }

        telemetry.addData("Power: ", liftPower);

        leftLiftMotor.setPower(liftPower);
        rightLiftMotor.setPower(liftPower);
        centerLiftMotor.setPower(liftPower);

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
        telemetry.addData("At position: ", atTargetPosition());
        telemetry.addData("Velocity: ", slidesEncoderMotor.getVelocity());

        liftPower = 0;
    }

    public void setLiftPower(double liftPower) {
        this.liftPower = liftPower;
    }

    public void setSlidesState(OuttakeSlidesStates newState) {

        if (newState == currentSlideState) {
            return;
        }

        if ((newState == OuttakeSlidesStates.SAMPLES || newState == OuttakeSlidesStates.SAMPLES_LOW) && robot.theIntake.requestedReturn) {
            robot.theCommandScheduler.scheduleCommand(
                    new SequentialCommandGroup(
                            new YieldCommand(() -> !robot.theIntake.isReturnRequested()),
                            new OneTimeCommand(() -> updateSlideState(newState))
                    )
            );
        } else {
            updateSlideState(newState);
        }
    }

    private void updateSlideState(OuttakeSlidesStates newState) {
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
        return -slidesEncoderMotor.getCurrentPosition() - positionDrift;
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

            robot.theCommandScheduler.scheduleCommand(
                    new SequentialCommandGroup(
                            new YieldCommand(50),
                            new OneTimeCommand(() -> setCurrentOuttakeState(OuttakeServoState.SPECIMEN_PICKUP_2)),
                            new YieldCommand(50),
                            new OneTimeCommand(() -> setSlidesState(OuttakeSlidesStates.SPECIMEN_TRANSFER)),
                            new YieldCommand(2000, this::atTargetPosition),
                            new OneTimeCommand(() -> setCurrentOuttakeState(OuttakeServoState.SPECIMEN_INITIAL)),
                            new OneTimeCommand(() -> setCurrentPivotState(OuttakePivotStates.SPECIMEN_DROP)),
                            new OneTimeCommand(() -> setCurrentRotationState(OuttakeRotationStates.ROTATED)),
                            new YieldCommand(200),
                            new OneTimeCommand(() -> setSlidesState(OuttakeSlidesStates.SPECIMENS)),
                            new OneTimeCommand(() -> setCurrentRotationState(OuttakeRotationStates.SPECIMEN_ROTATED))

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

    public OuttakeServoState getOuttakeServoState() {
        return currentOuttakeServoState;
    }
}
