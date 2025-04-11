package org.firstinspires.ftc.teamcode.utilities.robot.subsystems;

import android.graphics.Color;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.utilities.controltheory.motionprofiler.MotionProfile;
import org.firstinspires.ftc.teamcode.utilities.math.MathHelper;
import org.firstinspires.ftc.teamcode.utilities.robot.Alliance;
import org.firstinspires.ftc.teamcode.utilities.robot.Globals;
import org.firstinspires.ftc.teamcode.utilities.robot.RobotEx;
import org.firstinspires.ftc.teamcode.utilities.robot.command.framework.commandtypes.OneTimeCommand;
import org.firstinspires.ftc.teamcode.utilities.robot.command.framework.commandtypes.YieldCommand;
import org.firstinspires.ftc.teamcode.utilities.robot.command.framework.commandtypes.SequentialCommandGroup;
import org.mercurialftc.mercurialftc.util.hardware.cachinghardwaredevice.CachingDcMotorEX;
import org.mercurialftc.mercurialftc.util.hardware.cachinghardwaredevice.CachingServo;

@Config
public class Intake implements Subsystem {

    public enum LinkageStates {
        DEFAULT(0.364),
        AUTO_ROTATE(0.45),
        AUTO_EXTENSION(0.49),
        AUTO_EXTENSION_FURTHER(0.54),
        EXTENDED(0.62);

        public double position;

        LinkageStates(double position) {
            this.position = position;
        }

        public void setPosition(double position) {
            this.position = position;
        }

    }

    public enum IntakeState {
        DEFAULT(0.67),
        EJECT(0.65),
        TRANSFER(0.64),
        AUTO_DEFAULT(0.57),

        EXTENDED(0.80);

        public double position;

        IntakeState(double position) {
            this.position = position;
        }

        public void setPosition(double position) {
            this.position = position;
        }

    }

    public enum SampleHolderState {
        EXTENDED(0.05),
        DEFAULT(0.5);

        public double position;

        SampleHolderState(double position) {
            this.position = position;
        }

        public void setPosition(double position) {
            this.position = position;
        }
    }

    public enum LinkageHolderState {
        CLOSED(0.52),
        OPEN(0.46);

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
        HOLD(0.4),
        STATIONARY(0),
        SLOW_REVERSE(-0.65),
        HARD_REVERSE(-1),
        REVERSE(-0.8);

        public double position;

        IntakeMotorStates(double position) {
            this.position = position;
        }

        public void setPosition(double position) {
            this.position = position;
        }
    }

    public enum CowcatcherStates {
        ACTIVATED(0.98),
        DEFAULT(0.65);

        public double position;

        CowcatcherStates(double position) {
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
    public CowcatcherStates currentCowcatcherState = CowcatcherStates.DEFAULT;

    DcMotorEx activeMotor;

    public Servo leftServo;
    public Servo rightServo;

    public Servo leftDropdownServo;
    public Servo rightDropdownServo;

    Servo cowcatcherServo;

    Servo holderServo;

    Servo linkageLockServo;

    DigitalChannel intakeBreakbeam;
    DigitalChannel intakeColorSensorDigital0;
    DigitalChannel intakeColorSensorDigital1;

    AnalogInput linkageAnalog;

    RevColorSensorV3 intakeColorSensor1;
    RevColorSensorV3 intakeColorSensor2;

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
    boolean previousManual = false;
    boolean reverse = false;

    boolean disableAutomation = false;

    boolean disableOuttake = false;

    boolean lastBreakbeamState = false;
    boolean currentBreakbeamState = false;

    ElapsedTime containedTimer = new ElapsedTime();

    boolean scheduledAutomation = false;

    boolean requestedReturn = false;

    public double gain = 5.0;

    public static double aMax = 10;
    public static double vMax = 10;

    public static double velocity = 1;

    private MotionProfile profile = new MotionProfile(
            startLinkagePosition,
            startLinkagePosition,
            vMax,
            aMax
    );

    ElapsedTime linkageTimer = new ElapsedTime();

    Telemetry telemetry;

    RobotEx robot;

    @Override
    public void onInit(HardwareMap newHardwareMap, Telemetry newTelemetry) {
        leftServo = new CachingServo(newHardwareMap.get(Servo.class, "leftIntakeLinkageServo"), 1e-5);
        rightServo = new CachingServo(newHardwareMap.get(Servo.class, "rightIntakeLinkageServo"), 1e-5);
        leftDropdownServo = new CachingServo(newHardwareMap.get(Servo.class, "leftIntakeDropdownServo"), 1e-5);
        rightDropdownServo = new CachingServo(newHardwareMap.get(Servo.class, "rightIntakeDropdownServo"), 1e-5);
        holderServo = new CachingServo(newHardwareMap.get(Servo.class, "intakeHolderServo"), 1e-5);
        linkageLockServo = new CachingServo(newHardwareMap.get(Servo.class, "linkageLock"), 1e-5);
        activeMotor = new CachingDcMotorEX(newHardwareMap.get(DcMotorEx.class, "intakeMotor"), 1e-5);

        cowcatcherServo = new CachingServo(newHardwareMap.get(Servo.class, "cowcatcher"), 1e-5);

        intakeColorSensor1 = newHardwareMap.get(RevColorSensorV3.class, "intakeColorSensor1");
        intakeColorSensor2 = newHardwareMap.get(RevColorSensorV3.class, "intakeColorSensor2");
        intakeColorSensor1.setGain((float) gain);

        intakeColorSensorDigital0 = newHardwareMap.get(DigitalChannel.class, "digital0");
        intakeColorSensorDigital1 = newHardwareMap.get(DigitalChannel.class, "digital1");

        linkageAnalog = newHardwareMap.get(AnalogInput.class, "linkageAnalog");
        intakeBreakbeam = newHardwareMap.get(DigitalChannel.class, "intakeBreakbeam");

        cowcatcherServo.setDirection(Servo.Direction.REVERSE);

        leftServo.setDirection(Servo.Direction.REVERSE);
        rightServo.setDirection(Servo.Direction.REVERSE);

        leftDropdownServo.setDirection(Servo.Direction.REVERSE);
        rightDropdownServo.setDirection(Servo.Direction.FORWARD);

        activeMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        activeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        activeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        activeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        telemetry = newTelemetry;

        requestedReturn = false;

        manual = false;
        reverse = false;

        disableOuttake = false;

        lastBreakbeamState = false;
        currentBreakbeamState = false;

    }

    @Override
    public void onOpmodeStarted() {
        rebuildProfile(currentLinkageState.position);

        leftServo.setPosition(leftServo.getPosition() + 0.01);
        rightServo.setPosition(rightServo.getPosition() + 0.01);

        setIntakeMotorState(IntakeMotorStates.STATIONARY);
        setTargetLinkageState(LinkageStates.DEFAULT);
        setIntakeState(IntakeState.DEFAULT);

        robot = RobotEx.getInstance();
    }

    @Override
    public void onCyclePassed() {

        lastBreakbeamState = currentBreakbeamState;
        currentBreakbeamState = intakeColorSensor1.getDistance(DistanceUnit.INCH) < 1; // !intakeBreakbeam.getState();

        /*
        Add majority decision breakbeam
         */

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
        // telemetry.addData("Distance: ", intakeColorSensor.getDistance(DistanceUnit.INCH));
        telemetry.addData("Breakbeam state: ", !intakeBreakbeam.getState());
        telemetry.addData("Analog: ", linkageAnalog.getVoltage());
        telemetry.addData("At home: ", linkageAtHomeAnalog());
        telemetry.addData("Color Sensor Distance: ", intakeColorSensor1.getDistance(DistanceUnit.INCH));
        // telemetry.addData("Color Sensor Distance: ", intakeColorSensor2.getDistance(DistanceUnit.INCH));

        telemetry.addData("Possessed Color: ", sampleContained);

        // telemetry.addData("Servo latch position:", holderServo.getPosition());
        // telemetry.addData("Distance measured: ", intakeColorSensor.getDistance(DistanceUnit.INCH));

        NormalizedRGBA colors = intakeColorSensor1.getNormalizedColors();

        telemetry.addData("Red: ", colors.red);
        telemetry.addData("Green: ", colors.green);
        telemetry.addData("Blue: ", colors.blue);




        if (manual && !previousManual) {
            setIntakeMotorState(IntakeMotorStates.INTAKING);
            setIntakeState(IntakeState.EXTENDED);
        }

        if (currentBreakbeamState && !lastBreakbeamState && currentIntakeState != IntakeState.DEFAULT && linkageTimer.seconds() > 0.25) {
            containedTimer.reset();
            scheduledAutomation = true;
        }

        if (scheduledAutomation && !disableAutomation) {
            if (!currentBreakbeamState) {
                scheduledAutomation = false;
            }

            updatePossessedColor();

            if (!(containedTimer.seconds() > 0.05)) {
            } else if (containedTimer.seconds() > 0.5 || sampleContained != SampleContained.NONE) {
                scheduledAutomation = false;

                boolean wrongColor = false;

                if (Globals.ALLIANCE == Alliance.RED && sampleContained == SampleContained.BLUE) {
                    wrongColor = true;
                } else if (Globals.ALLIANCE == Alliance.BLUE && sampleContained == SampleContained.RED) {
                    wrongColor = true;
                }

                if (!wrongColor || disableOuttake) {
                    robot.theOpMode.gamepad1.rumble(500);
                    robot.theOpMode.gamepad2.rumble(500);
                    returnSlides();
                } else {
                    RobotEx.getInstance().theCommandScheduler.scheduleCommand(
                            new SequentialCommandGroup(
                                    new OneTimeCommand(this::reverseIntake),
                                    new YieldCommand(1500)
                            )
                    );
                }
            }
        }

        if (linkageAtHome() && !manual) {
            setLinkageHolderState(LinkageHolderState.CLOSED);
        } else {
            setLinkageHolderState(LinkageHolderState.OPEN);
        }

        leftDropdownServo.setPosition(currentIntakeState.position);
        rightDropdownServo.setPosition(currentIntakeState.position);

        holderServo.setPosition(currentSampleHolderState.position);

        linkageLockServo.setPosition(currentLinkageHolderState.position);

        leftServo.setPosition(currentTargetPosition);
        rightServo.setPosition(currentTargetPosition);

        activeMotor.setPower(currentIntakeMotorState.position);

        cowcatcherServo.setPosition(currentCowcatcherState.position);

        telemetry.addData("Intake State: ", currentLinkageState);
        telemetry.addData("Linkage Holder State: ", currentLinkageHolderState);
        telemetry.addData("Intake Motor State: ", currentIntakeMotorState);
        // telemetry.addData("Intake Current: ", activeMotor.getCurrent(CurrentUnit.MILLIAMPS));
        // telemetry.addData("Linkage Position: ", currentLinkageState.position);
        telemetry.addData("Drop Down State: ", currentIntakeState);
        telemetry.addData("Cowcatcher State: ", currentCowcatcherState);
        // telemetry.addData("Cowcatcher position: ", cowcatcherServo.getPosition());
        telemetry.addData("Holder State: ", currentSampleHolderState);
        // telemetry.addData("Distance: ", intakeColorSensor.getDistance(DistanceUnit.INCH));
        // telemetry.addData("Color Sensor digital 0: ", intakeColorSensorDigital0.getState());
        // telemetry.addData("Color sensor digital 1: ", intakeColorSensorDigital1.getState());
        // telemetry.addData("Servo position: ", holderServo.getPosition());

        reverse = false;
        previousManual = manual;
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

        profile = new MotionProfile(
                getCurrentPosition(),
                targetPosition,
                vMax,
                aMax
        );

        manual = false;

        linkageTimer.reset();

    }

    public void setTargetLinkageState(LinkageStates newState) {

        if (newState == currentLinkageState && !manual) {
            return;
        }

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

        if (newState == IntakeMotorStates.REVERSE || newState == IntakeMotorStates.SLOW_REVERSE) {
            manual = false;
        }

        currentIntakeMotorState = newState;
    }

    public void reverseIntake() {
        RobotEx.getInstance().theCommandScheduler.scheduleCommand(
            new SequentialCommandGroup(
                    new OneTimeCommand(() -> setTargetHolderState(SampleHolderState.DEFAULT)),
                    new OneTimeCommand(() -> setIntakeState(IntakeState.EJECT)),
                    new YieldCommand(200),
                    new OneTimeCommand(() -> setIntakeMotorState(IntakeMotorStates.REVERSE))
            )
        );
    }

    public void updatePossessedColor() {


        float[] hsvValues = new float[3];

        NormalizedRGBA colors = intakeColorSensor1.getNormalizedColors();
        // Color.colorToHSV(colors.toColor(), hsvValues);

        double green = colors.green;
        double red = colors.red;
        double blue = colors.blue;

        /* Use telemetry to display feedback on the driver station. We show the red, green, and blue
         * normalized values from the sensor (in the range of 0 to 1), as well as the equivalent
         * HSV (hue, saturation and value) values. See http://web.archive.org/web/20190311170843/https://infohost.nmt.edu/tcc/help/pubs/colortheory/web/hsv.html
         * for an explanation of HSV color. */

        // Update the hsvValues array by passing it to Color.colorToHSV()
        Color.colorToHSV(colors.toColor(), hsvValues);


        // no sample:
        // red: 0.0313
        // blue: 0.0972
        // green: 0.0894

        // yellow sample:
        // red: 0.1512
        // blue: 0.1225
        // green: 0.2209

        // blue sample:
        // red: 0.0428
        // blue: 0.1661
        // green: 0.1143

        // red sample:
        // red: 0.1056
        // blue: 0.1102
        // green: 0.1186

        System.out.println("Green: " + green);
        System.out.println("Red: " + red);
        System.out.println("Blue: " + blue);
        System.out.println("Hue: " + hsvValues[0]);
        System.out.println("Saturation: " + hsvValues[1]);
        System.out.println("Value: " + hsvValues[2]);

        double hue = hsvValues[0];
        double saturation = hsvValues[1];
        double value = hsvValues[2];

        if (hue > 180 && hue < 300) {
            sampleContained = SampleContained.BLUE;
        } else if (hue < 120 && hue > 50) {
            sampleContained = SampleContained.YELLOW;
        } else {
            sampleContained = SampleContained.RED;
        }
        /*
        if (green > 0.002 && red > 0.002) {
            sampleContained = SampleContained.YELLOW;
        } else if (red > 0.001 && green < 0.001 && blue < 0.0015){
            sampleContained = SampleContained.RED;
        } else if (blue > 0.001 && red < 0.001 && green < 0.001) {
            sampleContained = SampleContained.BLUE;
        } else {
            sampleContained = SampleContained.NONE;
        }

         */


        if (!containsSampleColorSensor()) {
            System.out.println("No Sample Detected by Distance" + intakeColorSensor2.getDistance(DistanceUnit.INCH));
            sampleContained = SampleContained.NONE;
        }

        System.out.println(sampleContained);
        /*
        double buffer = 0.04;

        // Check for yellow sample
        if (Math.abs(red - 0.1512) <= buffer && Math.abs(blue - 0.1225) <= buffer && Math.abs(green - 0.2209) <= buffer) {
            sampleContained = SampleContained.YELLOW;
        }
        // Check for blue sample
        else if (Math.abs(red - 0.0428) <= buffer && Math.abs(blue - 0.1661) <= buffer && Math.abs(green - 0.1143) <= buffer) {
            sampleContained = SampleContained.BLUE;
        }
        // Check for red sample
        else if (Math.abs(red - 0.1056) <= buffer && Math.abs(blue - 0.1102) <= buffer && Math.abs(green - 0.1186) <= buffer) {
            sampleContained = SampleContained.RED;
        }
        // No sample
        else {
            sampleContained = SampleContained.NONE;
        }

         */


        /*
        if ((green + red + blue) < 900) {
            sampleContained = SampleContained.NONE;
            return;
        }

         */

        /*

        if (red > green && red > blue) {
            sampleContained = SampleContained.RED;
        } else if (blue > green && blue > red) {
            sampleContained = SampleContained.BLUE;
        } else {
            sampleContained = SampleContained.YELLOW;
        }

         */
        /*

        if (green > 17000) {
            sampleContained = SampleContained.YELLOW;
        } else if (red > 7500) {
            sampleContained = SampleContained.RED;
        } else if (blue > 14000) {
            sampleContained = SampleContained.BLUE;
        } else {
            sampleContained = SampleContained.NONE;
        }
         */

        /*
        if (green > red && green > blue) {
            sampleContained = SampleContained.YELLOW;
        } else if (red > green && red > blue) {
            sampleContained = SampleContained.RED;
        } else if (blue > green && blue > red) {
            sampleContained = SampleContained.BLUE;
        } else {
            sampleContained = SampleContained.NONE;
        }

         */
    }

    public boolean getCurrentPositionFromAnalog() {
        return linkageAnalog.getVoltage() > 1.5;
    }

    public boolean linkageAtHome() {
        return linkageAtTargetPosition() && currentLinkageState == LinkageStates.DEFAULT; // linkageAnalog.getVoltage() < 0.165 && currentLinkageState == LinkageStates.DEFAULT; // linkageAtTargetPosition() && currentLinkageState == LinkageStates.DEFAULT;
    }

    public boolean linkageAtHomeAnalog() {
        return linkageAnalog.getVoltage() < 1.390 && currentLinkageState == LinkageStates.DEFAULT;
    }

    public void setLinkageHolderState(LinkageHolderState newState) {
        currentLinkageHolderState = newState;
    }

    public boolean linkageAtTargetPosition() {
        return profile.getDuration() < linkageTimer.seconds()+0.1;
    }

    public void returnSlides() {
        if (requestedReturn) {
            return;
        }

        requestedReturn = true;
        manual = false;

        robot.theCommandScheduler.scheduleCommand(
                new SequentialCommandGroup(
                        new OneTimeCommand(() -> {
                            if (robot.theOuttake.getSlidesState() == Outtake.OuttakeSlidesStates.DEFAULT && robot.theOuttake.atTargetPosition()) {
                                robot.theOuttake.setCurrentClawState(Outtake.OuttakeClawStates.DEFAULT);
                            }
                        }),
                        new OneTimeCommand(() -> setTargetHolderState(SampleHolderState.EXTENDED)),
                        new YieldCommand(0),
                        new OneTimeCommand(() -> setIntakeState(IntakeState.DEFAULT)),
                        new YieldCommand(100), // Wait for holder servo to fully actuate
                        new OneTimeCommand(() -> {
                            if (robot.theOuttake.getSlidesState() == Outtake.OuttakeSlidesStates.DEFAULT && robot.theOuttake.atTargetPosition()) {
                                robot.theOuttake.setCurrentClawState(Outtake.OuttakeClawStates.DEFAULT);
                            }
                        }),
                        new OneTimeCommand(() -> setTargetLinkageState(LinkageStates.DEFAULT)),
                        new OneTimeCommand(() -> setIntakeMotorState(IntakeMotorStates.REVERSE)),
                        new YieldCommand(1500, this::linkageAtHomeAnalog), // Wait for slides to return
                        new OneTimeCommand(() -> setIntakeMotorState(IntakeMotorStates.STATIONARY)),
                        new YieldCommand(75),
                        new YieldCommand(robot.theOuttake::atTargetPosition),
                        new OneTimeCommand(() -> robot.theOuttake.setCurrentClawState(Outtake.OuttakeClawStates.CLOSED)),
                        new YieldCommand(50), // Wait for claw to close
                        new OneTimeCommand(() -> setIntakeMotorState(IntakeMotorStates.STATIONARY)),
                        new OneTimeCommand(() -> setTargetHolderState(SampleHolderState.DEFAULT)),
                        new YieldCommand(50),
                        new OneTimeCommand(() -> requestedReturn = false)
                )
        );

    }

    public void setCurrentCowcatcherState(CowcatcherStates newState) {
        currentCowcatcherState = newState;
    }

    public boolean containsSample() {
        return (activeMotor.getCurrent(CurrentUnit.MILLIAMPS) > 1000) || currentBreakbeamState;
    }

    public boolean containsSampleColorSensor() {
        return currentBreakbeamState;
    }

    public void setDisableOuttake(boolean disable) {
        disableOuttake = disable;
    }

    public boolean isReturnRequested() {
        return requestedReturn;
    }

    public void triggerCowcatcher() {
        robot.theCommandScheduler.scheduleCommand(
                new SequentialCommandGroup(
                        new OneTimeCommand(() -> robot.theIntake.setCurrentCowcatcherState(Intake.CowcatcherStates.ACTIVATED)),
                        new YieldCommand(250),
                        new OneTimeCommand(() -> robot.theIntake.setCurrentCowcatcherState(Intake.CowcatcherStates.DEFAULT)))
        );
    }

    public void setAutomation(boolean bool) {
        disableAutomation = !bool;
    }

}
