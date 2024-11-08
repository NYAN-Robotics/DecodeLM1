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
        DEFAULT(0.56),
        EXTENDED(0.76),
        UP(0.45);

        public double position;

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

    public static double startPosition = LinkageStates.DEFAULT.position;
    public static double extendedPosition = LinkageStates.EXTENDED.position;

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
    public void onInit(HardwareMap newHardwareMap, Telemetry newTelemetry) {
        leftServo = new CachingServo(newHardwareMap.get(Servo.class, "leftIntakeServo"), 1e-5);
        rightServo = new CachingServo(newHardwareMap.get(Servo.class, "rightIntakeServo"), 1e-5);
        leftDropdownServo = new CachingServo(newHardwareMap.get(Servo.class, "leftDropdownServo"), 1e-5);
        rightDropdownServo = new CachingServo(newHardwareMap.get(Servo.class, "rightDropdownServo"), 1e-5);
        activeMotor = new CachingDcMotorEX(newHardwareMap.get(DcMotorEx.class, "intakeMotor"), 1e-5);

        leftServo.setDirection(Servo.Direction.REVERSE);
        rightServo.setDirection(Servo.Direction.REVERSE);
        leftDropdownServo.setDirection(Servo.Direction.REVERSE);
        rightDropdownServo.setDirection(Servo.Direction.FORWARD);

        activeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        activeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        activeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        activeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        activeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        telemetry = newTelemetry;
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
            activeMotor.setPower(-0.5);
        } else if (currentIntakeState == IntakeState.EXTENDED && getCurrentPosition()-0.01 > LinkageStates.DEFAULT.position) {
            activeMotor.setPower(1);
        } else if (RobotEx.getInstance().outtake.slidesTimer.seconds() < 1) {
            activeMotor.setPower(0.5);
        } else if (RobotEx.getInstance().outtake.getClawState() == Outtake.OuttakeClawStates.CLOSED) {
            activeMotor.setPower(0.3);
        } else {
            activeMotor.setPower(0.);
        }

        if (currentLinkageState == LinkageStates.DEFAULT && profile.getDuration() <= linkageTimer.seconds() && currentIntakeState == IntakeState.DEFAULT) {
            this.setIntakeState(IntakeState.EXTENDED);
        }


        leftDropdownServo.setPosition(currentIntakeState.position);
        rightDropdownServo.setPosition(currentIntakeState.position);

        leftServo.setPosition(currentTargetPosition);
        rightServo.setPosition(currentTargetPosition);

        telemetry.addData("Intake State: ", currentLinkageState);
        telemetry.addData("Linkage Position: ", currentLinkageState.position);
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
