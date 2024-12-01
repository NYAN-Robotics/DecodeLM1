package org.firstinspires.ftc.teamcode.utilities.robot.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.utilities.controltheory.motionprofiler.MotionProfile;
import org.firstinspires.ftc.teamcode.utilities.math.MathHelper;
import org.firstinspires.ftc.teamcode.utilities.robot.Alliance;
import org.firstinspires.ftc.teamcode.utilities.robot.Globals;
import org.firstinspires.ftc.teamcode.utilities.robot.RobotEx;
import org.mercurialftc.mercurialftc.util.hardware.cachinghardwaredevice.CachingDcMotorEX;
import org.mercurialftc.mercurialftc.util.hardware.cachinghardwaredevice.CachingServo;

@Config
public class Intake implements Subsystem {

    public enum LinkageStates {
        DEFAULT(0),
        EXTENDED(0.2);

        public double position;

        LinkageStates(double position) {
            this.position = position;
        }

        public void setPosition(double position) {
            this.position = position;
        }

    }

    public enum IntakeState {
        PUSH_DOWN(1),
        DEFAULT(0.6),
        EXTENDED(0.72),
        UP(0.45);

        public double position;

        IntakeState(double position) {
            this.position = position;
        }

        public void setPosition(double position) {
            this.position = position;
        }

    }

    public enum SampleHolderState {
        EXTENDED(0.53),
        DEFAULT(0.9);

        public double position;

        SampleHolderState(double position) {
            this.position = position;
        }

        public void setPosition(double position) {
            this.position = position;
        }
    }

    public enum LinkageHolderState {
        CLOSED(0.18),
        OPEN(0);

        public double position;

        LinkageHolderState(double position) {
            this.position = position;
        }

        public void setPosition(double position) {
            this.position = position;
        }
    }

    public enum IntakeMotorStates {
        INTAKING(1),
        HOLD(0.2),
        STATIONARY(0),
        SLOW_REVERSE(-0.4),
        REVERSE(-0.75);

        public double position;

        IntakeMotorStates(double position) {
            this.position = position;
        }

        public void setPosition(double position) {
            this.position = position;
        }
    }

    public enum SampleContained {
        NONE,
        BLUE,
        RED,
        YELLOW
    }


    public LinkageStates currentLinkageState = LinkageStates.DEFAULT;
    public IntakeState currentIntakeState = IntakeState.DEFAULT;
    public SampleHolderState currentSampleHolderState = SampleHolderState.DEFAULT;
    public LinkageHolderState currentLinkageHolderState = LinkageHolderState.OPEN;
    public IntakeMotorStates currentIntakeMotorState = IntakeMotorStates.STATIONARY;
    public SampleContained sampleContained = SampleContained.NONE;

    DcMotorEx activeMotor;

    Servo leftServo;
    Servo rightServo;

    Servo leftDropdownServo;
    Servo rightDropdownServo;

    Servo holderServo;

    Servo linkageLockServo;

    DigitalChannel intakeBreakbeam;

    AnalogInput linkageAnalog;

    RevColorSensorV3 intakeColorSensor;

    public static double startLinkagePosition = LinkageStates.DEFAULT.position;
    public static double extendedLinkagePosition = LinkageStates.EXTENDED.position;

    public static double defaultIntakePosition = IntakeState.DEFAULT.position;
    public static double extendedIntakePosition = IntakeState.EXTENDED.position;

    public static double defaultLinkageHolderPosition = LinkageHolderState.OPEN.position;
    public static double extendedLinkageHolderPosition = LinkageHolderState.CLOSED.position;

    public static double defaultSampleHolderPosition = SampleHolderState.DEFAULT.position;
    public static double extendedSampleHolderPosition = SampleHolderState.EXTENDED.position;

    private double targetPosition = startLinkagePosition;

    boolean manual = false;
    boolean reverse = false;

    boolean lastBreakbeamState = false;
    boolean currentBreakbeamState = false;

    public static double aMax = 1;
    public static double vMax = 5;

    public static double velocity = 1;

    private MotionProfile profile = new MotionProfile(
            startLinkagePosition,
            startLinkagePosition,
            vMax,
            aMax
    );

    ElapsedTime linkageTimer = new ElapsedTime();

    Telemetry telemetry;

    @Override
    public void onInit(HardwareMap newHardwareMap, Telemetry newTelemetry) {
        leftServo = new CachingServo(newHardwareMap.get(Servo.class, "leftIntakeLinkageServo"), 1e-5);
        rightServo = new CachingServo(newHardwareMap.get(Servo.class, "rightIntakeLinkageServo"), 1e-5);
        leftDropdownServo = new CachingServo(newHardwareMap.get(Servo.class, "leftIntakeDropdownServo"), 1e-5);
        rightDropdownServo = new CachingServo(newHardwareMap.get(Servo.class, "rightIntakeDropdownServo"), 1e-5);
        holderServo = new CachingServo(newHardwareMap.get(Servo.class, "intakeHolderServo"), 1e-5);
        linkageLockServo = new CachingServo(newHardwareMap.get(Servo.class, "linkageLock"), 1e-5);
        activeMotor = new CachingDcMotorEX(newHardwareMap.get(DcMotorEx.class, "intakeMotor"), 1e-5);

        intakeColorSensor = newHardwareMap.get(RevColorSensorV3.class, "intakeColorSensor");

        linkageAnalog = newHardwareMap.get(AnalogInput.class, "linkageAnalog");
        intakeBreakbeam = newHardwareMap.get(DigitalChannel.class, "intakeBreakbeam");

        leftServo.setDirection(Servo.Direction.REVERSE);
        rightServo.setDirection(Servo.Direction.REVERSE);
        leftDropdownServo.setDirection(Servo.Direction.REVERSE);
        rightDropdownServo.setDirection(Servo.Direction.FORWARD);

        activeMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        activeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        activeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        activeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);


        telemetry = newTelemetry;
    }

    @Override
    public void onOpmodeStarted() {
        rebuildProfile(currentLinkageState.position);

        /*
        intakeColorSensor.initialize();
        intakeColorSensor.enableLed(true);
         */

        setIntakeMotorState(IntakeMotorStates.STATIONARY);
        setTargetLinkageState(LinkageStates.DEFAULT);
        setIntakeState(IntakeState.DEFAULT);
    }

    @Override
    public void onCyclePassed() {

        lastBreakbeamState = currentBreakbeamState;
        currentBreakbeamState = !intakeBreakbeam.getState();

        LinkageStates.DEFAULT.setPosition(startLinkagePosition);
        LinkageStates.EXTENDED.setPosition(extendedLinkagePosition);

        IntakeState.DEFAULT.setPosition(defaultIntakePosition);
        IntakeState.EXTENDED.setPosition(extendedIntakePosition);

        SampleHolderState.DEFAULT.setPosition(defaultSampleHolderPosition);
        SampleHolderState.EXTENDED.setPosition(extendedSampleHolderPosition);

        LinkageHolderState.OPEN.setPosition(defaultLinkageHolderPosition);
        LinkageHolderState.CLOSED.setPosition(extendedLinkageHolderPosition);

        double currentTargetPosition = getCurrentPosition();


        // telemetry.addData("Current Position!!!:", getCurrentPosition());
        // telemetry.addData("Linkage Position: ", LinkageStates.DEFAULT.position);
        // telemetry.addData("Manual: ", getCurrentPosition() != LinkageStates.DEFAULT.position);
        // telemetry.addData("Sample Holder State: ", currentSampleHolderState);
        // telemetry.addData("Linkage Holder State: ", currentLinkageHolderState);
        telemetry.addData("Breakbeam state: ", currentBreakbeamState);
        telemetry.addData("Analog: ", linkageAnalog.getVoltage());
        telemetry.addData("Possessed Color: ", sampleContained);
        /*
        telemetry.addData("Red: ", intakeColorSensor.red());
        telemetry.addData("Green: ", intakeColorSensor.green());
        telemetry.addData("Blue: ", intakeColorSensor.blue());



         */
        /*
        if (reverse) {
            activeMotor.setPower(-1);
        } else if (currentIntakeState == IntakeState.EXTENDED && getCurrentPosition()-0.01 > LinkageStates.DEFAULT.position) {
            activeMotor.setPower(1);
        } else if (RobotEx.getInstance().outtake.slidesTimer.seconds() < 1) {
            activeMotor.setPower(0.5);
        } else if (RobotEx.getInstance().outtake.getClawState() == Outtake.OuttakeClawStates.CLOSED) {
            activeMotor.setPower(0.3);
        } else {
            activeMotor.setPower(0.);
        }

         */

        if (currentBreakbeamState && !lastBreakbeamState) {
            updatePossessedColor();

            boolean wrongColor = false;

            if (Globals.ALLIANCE == Alliance.RED && sampleContained == SampleContained.BLUE) {
                wrongColor = true;
            } else if (Globals.ALLIANCE == Alliance.BLUE && sampleContained == SampleContained.RED) {
                wrongColor = true;

            }

            if (!wrongColor) {
                setTargetLinkageState(LinkageStates.DEFAULT);
                setTargetHolderState(SampleHolderState.EXTENDED);
                setIntakeState(IntakeState.DEFAULT);
                setIntakeMotorState(IntakeMotorStates.SLOW_REVERSE);
            } else {
                setTargetHolderState(SampleHolderState.DEFAULT);
                setIntakeMotorState(IntakeMotorStates.REVERSE);
                setIntakeState(IntakeState.DEFAULT);
            }
        }


        leftDropdownServo.setPosition(currentIntakeState.position);
        rightDropdownServo.setPosition(currentIntakeState.position);

        holderServo.setPosition(currentSampleHolderState.position);

        linkageLockServo.setPosition(currentLinkageHolderState.position);

        leftServo.setPosition(currentTargetPosition);
        rightServo.setPosition(currentTargetPosition);

        activeMotor.setPower(currentIntakeMotorState.position);

        telemetry.addData("Intake Motor Current: ", activeMotor.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("Intake State: ", currentLinkageState);
        telemetry.addData("Linkage Position: ", currentLinkageState.position);
        telemetry.addData("Drop Down State: ", currentIntakeState);
        telemetry.addData("Holder State: ", holderServo);

        reverse = false;
    }

    public void setTargetPosition(double newPosition) {
        targetPosition = MathHelper.clamp(newPosition, LinkageStates.DEFAULT.position, LinkageStates.EXTENDED.position);

        manual = true;
    }

    public double getCurrentPosition() {
        if (manual || profile == null) {
            return targetPosition;
        }

        return profile.getPositionFromTime(linkageTimer.seconds());
    }

    private void rebuildProfile(double targetPosition) {

        manual = false;


        profile = new MotionProfile(
                getCurrentPosition(),
                targetPosition,
                vMax,
                aMax
        );

        linkageTimer.reset();

    }

    public void setTargetLinkageState(LinkageStates newState) {
        rebuildProfile(newState.position);

        currentLinkageState = newState;
    }

    public void setTargetHolderState(SampleHolderState newState) {
        currentSampleHolderState = newState;
    }

    public void incrementPositionByVelocity(double amount, double dt) {

        targetPosition = getCurrentPosition() + amount * velocity * dt;

        targetPosition = MathHelper.clamp(targetPosition, LinkageStates.DEFAULT.position, LinkageStates.EXTENDED.position);

        manual = true;

    }

    public void setIntakeState(IntakeState newState) {
        currentIntakeState = newState;
    }

    public void setIntakeMotorState(IntakeMotorStates newState) {
        currentIntakeMotorState = newState;
    }

    public void reverseIntake() {
        reverse = true;
    }

    public void updatePossessedColor() {
        int green = intakeColorSensor.green();
        int red = intakeColorSensor.red();
        int blue = intakeColorSensor.blue();

        if (green > red && green > blue) {
            sampleContained = SampleContained.YELLOW;
        } else if (red > green && red > blue) {
            sampleContained = SampleContained.RED;
        } else if (blue > green && blue > red) {
            sampleContained = SampleContained.BLUE;
        } else {
            sampleContained = SampleContained.NONE;
        }
    }
}
