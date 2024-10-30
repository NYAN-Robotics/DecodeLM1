package org.firstinspires.ftc.teamcode.utilities.robot.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.utilities.controltheory.motionprofiler.MotionProfile;
import org.firstinspires.ftc.teamcode.utilities.math.MathHelper;
import org.firstinspires.ftc.teamcode.utilities.robot.RobotEx;
import org.mercurialftc.mercurialftc.util.hardware.cachinghardwaredevice.CachingDcMotorEX;
import org.mercurialftc.mercurialftc.util.hardware.cachinghardwaredevice.CachingServo;

@Config
public class Intake implements Subsystem {

    public enum LinkageStates {
        DEFAULT(0.075),
        EXTENDED(0.2);

        public double position;

        // Constructor
        LinkageStates(double position) {
            this.position = position;
        }

        public void setPosition(double position) {
            this.position = position;
        }

    }

    public enum IntakeState {
        DEFAULT(0.56),
        EXTENDED(0.76);

        public double position;

        // Constructor
        IntakeState(double position) {
            this.position = position;
        }

        public void setPosition(double position) {
            this.position = position;
        }

    }

    public LinkageStates currentLinkageState = LinkageStates.DEFAULT;
    public IntakeState currentIntakeState = IntakeState.DEFAULT;

    DcMotorEx activeMotor;

    Servo leftServo;
    Servo rightServo;
    Servo leftDropdownServo;
    Servo rightDropdownServo;

    public static double startPosition = 0.07;
    public static double extendedPosition = 0.27;

    public static double defaultIntakePosition = IntakeState.DEFAULT.position;
    public static double extendedIntakePosition = IntakeState.EXTENDED.position;

    private double targetPosition = startPosition;

    boolean manual = false;
    boolean reverse = false;

    public static double aMax = 2.5;
    public static double vMax = 5;

    public static double velocity = 1;

    private MotionProfile profile = new MotionProfile(
            startPosition,
            startPosition,
            vMax,
            aMax
    );

    ElapsedTime linkageTimer = new ElapsedTime();

    Telemetry telemetry;

    @Override
    public void onInit(HardwareMap hardwareMap, Telemetry telemetry) {
        leftServo = new CachingServo(hardwareMap.get(Servo.class, "leftIntakeServo"), 1e-5);
        rightServo = new CachingServo(hardwareMap.get(Servo.class, "rightIntakeServo"), 1e-5);
        leftDropdownServo = new CachingServo(hardwareMap.get(Servo.class, "leftDropdownServo"), 1e-5);
        rightDropdownServo = new CachingServo(hardwareMap.get(Servo.class, "rightDropdownServo"), 1e-5);
        activeMotor = new CachingDcMotorEX(hardwareMap.get(DcMotorEx.class, "intakeMotor"), 1e-5);

        leftServo.setDirection(Servo.Direction.REVERSE);
        rightServo.setDirection(Servo.Direction.REVERSE);
        leftDropdownServo.setDirection(Servo.Direction.REVERSE);
        rightDropdownServo.setDirection(Servo.Direction.FORWARD);

        activeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        activeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        activeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        activeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        activeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        this.telemetry = telemetry;
    }

    @Override
    public void onOpmodeStarted() {
        rebuildProfile(currentLinkageState.position);
    }

    @Override
    public void onCyclePassed() {

        LinkageStates.DEFAULT.setPosition(startPosition);
        LinkageStates.EXTENDED.setPosition(extendedPosition);

        IntakeState.DEFAULT.setPosition(defaultIntakePosition);
        IntakeState.EXTENDED.setPosition(extendedIntakePosition);

        double currentTargetPosition = getCurrentPosition();

        telemetry.addData("Current Position!!!:", getCurrentPosition());
        telemetry.addData("Linkage Position: ", LinkageStates.DEFAULT.position);
        telemetry.addData("Manual: ", getCurrentPosition() != LinkageStates.DEFAULT.position);

        if (reverse) {
            activeMotor.setPower(-0.9);
        } else if (currentIntakeState == IntakeState.EXTENDED && getCurrentPosition()-0.01 > LinkageStates.DEFAULT.position) {
            activeMotor.setPower(0.9);
        } else if (RobotEx.getInstance().outtake.slidesTimer.seconds() < 0.5) {
            activeMotor.setPower(0.1);
        } else {
            activeMotor.setPower(0);
        }

        if (currentLinkageState == LinkageStates.DEFAULT && profile.getDuration() <= linkageTimer.seconds()) {
            this.setIntakeState(IntakeState.EXTENDED);
        }


        leftDropdownServo.setPosition(currentIntakeState.position);
        rightDropdownServo.setPosition(currentIntakeState.position);

        leftServo.setPosition(currentTargetPosition);
        rightServo.setPosition(currentTargetPosition);

        telemetry.addData("Intake State: ", currentLinkageState);
        telemetry.addData("Drop Down State: ", currentIntakeState);

        reverse = false;
    }

    public void setTargetPosition(double newPosition) {
        targetPosition = MathHelper.clamp(newPosition, 0, 1);

        manual = true;
    }

    public double getCurrentPosition() {
        if (manual || profile == null) {
            return targetPosition;
        }

        return this.profile.getPositionFromTime(linkageTimer.seconds());
    }

    private void rebuildProfile(double targetPosition) {

        manual = false;


        this.profile = new MotionProfile(
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

    public void incrementPositionByVelocity(double amount, double dt) {
        manual = true;

        targetPosition += amount * velocity * dt;

        targetPosition = MathHelper.clamp(targetPosition, LinkageStates.DEFAULT.position, LinkageStates.EXTENDED.position);
    }

    public void setIntakeState(IntakeState newState) {
        currentIntakeState = newState;
    }

    public void reverseIntake() {
        reverse = true;
    }
}
