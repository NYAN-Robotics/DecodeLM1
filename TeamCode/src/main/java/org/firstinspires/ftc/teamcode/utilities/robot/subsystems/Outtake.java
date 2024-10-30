package org.firstinspires.ftc.teamcode.utilities.robot.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Outtake implements Subsystem {

    public enum OuttakeSlidesStates {
        DEFAULT(0),
        SAMPLES(600),
        SPECIMENS(400);

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
        EXTENDED(0);

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
        DEFAULT(0.8),
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

    Servo leftOuttakeServo;
    Servo rightOuttakeServo;

    Servo rotationOuttakeServo;
    Servo clawServo;

    DcMotorEx intakeMotor;

    OuttakeServoState currentOuttakeServoState = OuttakeServoState.DEFAULT;
    OuttakeRotationStates currentRotationState = OuttakeRotationStates.DEFAULT;
    OuttakeClawStates currentClawState = OuttakeClawStates.DEFAULT;

    double liftPower = 0;

    Telemetry telemetry;

    @Override
    public void onInit(HardwareMap hardwareMap, Telemetry telemetry) {
        leftLiftMotor = hardwareMap.get(DcMotorEx.class, "leftLiftMotor");
        rightLiftMotor = hardwareMap.get(DcMotorEx.class, "rightLiftMotor");

        leftOuttakeServo = hardwareMap.get(Servo.class, "leftOuttakeServo");
        rightOuttakeServo = hardwareMap.get(Servo.class, "rightOuttakeServo");

        rotationOuttakeServo = hardwareMap.get(Servo.class, "wrist");

        clawServo = hardwareMap.get(Servo.class, "claw");

        intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");

        leftOuttakeServo.setDirection(Servo.Direction.FORWARD); // dir
        rightOuttakeServo.setDirection(Servo.Direction.REVERSE); // Dir

        leftLiftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightLiftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        leftLiftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rightLiftMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        leftLiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightLiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

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
        leftLiftMotor.setPower(liftPower);
        rightLiftMotor.setPower(liftPower);

        leftOuttakeServo.setPosition(currentOuttakeServoState.position);
        rightOuttakeServo.setPosition(currentOuttakeServoState.position);

        rotationOuttakeServo.setPosition(currentRotationState.position);
        clawServo.setPosition(currentClawState.position);

        telemetry.addData("Outtake Servo State: ", currentOuttakeServoState);
        telemetry.addData("Outtake Position: ", currentOuttakeServoState.position);

        telemetry.addData("Rotation State: ", currentRotationState);
        telemetry.addData("Claw State: ", currentClawState);
        telemetry.addData("Lift Power: ", liftPower);
        telemetry.addData("Position: ", leftLiftMotor.getCurrentPosition());
        liftPower = 0;
    }

    public void setLiftPower(double liftPower) {
        this.liftPower = liftPower;
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
}
