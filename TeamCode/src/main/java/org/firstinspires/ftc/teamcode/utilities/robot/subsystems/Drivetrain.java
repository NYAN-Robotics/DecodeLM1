package org.firstinspires.ftc.teamcode.utilities.robot.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.utilities.controltheory.feedback.GeneralPIDController;
import org.firstinspires.ftc.teamcode.utilities.math.MathHelper;
import org.firstinspires.ftc.teamcode.utilities.math.linearalgebra.Pose;
import org.firstinspires.ftc.teamcode.utilities.physics.states.MecanumWheelState;
import org.firstinspires.ftc.teamcode.utilities.robot.RobotEx;
import org.firstinspires.ftc.teamcode.utilities.robot.extensions.MotorGroup;
import org.mercurialftc.mercurialftc.util.hardware.cachinghardwaredevice.CachingDcMotorEX;

/**
 * Robot Drivetrain
 */
// LF - 0
// RF - 2
// RB - 1
// LB - 3
@Config
public class Drivetrain implements Subsystem {

    public enum TurnDirection {
        LEFT, RIGHT
    }

    private RobotEx robotInstance;
    private Localizer theLocalizer;

    MotorGroup<DcMotorEx> drivetrainMotorGroup;
    private DcMotorEx[] drivetrainMotors;

    public DcMotorEx rightFrontMotor;
    public DcMotorEx leftFrontMotor;
    public DcMotorEx leftBackMotor;
    public DcMotorEx rightBackMotor;


    private boolean enableAntiTip = false;
    private boolean enableHeadingRetention = false;

    public GeneralPIDController xController = new GeneralPIDController(0.2, 0, 1, 0);
    public GeneralPIDController yController = new GeneralPIDController(0.2, 0, 1, 0);
    public GeneralPIDController headingController = new GeneralPIDController(1.5, 0, 0, 0);

    public GeneralPIDController headingPID = new GeneralPIDController(1.5, 0, 20, 0);
    public GeneralPIDController profiledTurningPID = new GeneralPIDController(1.5, 0, 40, 0);

    public GeneralPIDController translationalPID = new GeneralPIDController(1, 0, 0, 0);

    public GeneralPIDController tiltPID = new GeneralPIDController(0.3, 0, 0, 0);
    public GeneralPIDController yawPID = new GeneralPIDController(0.3, 0, 0, 0);

    private Telemetry telemetry;

    private double rightFrontPower = 0;
    private double leftFrontPower = 0;
    private double leftBackPower = 0;
    private double rightBackPower = 0;

    private double lastRightFrontPower = 0;
    private double lastLeftFrontPower = 0;
    private double lastLeftBackPower = 0;
    private double lastRightBackPower = 0;


    private double lastX = 0;
    private double lastY = 0;
    private double lastRot = 0;

    private double weight = 1;

    public static double kP = 2;
    public static double kI = 0;
    public static double kD = 40;

    private double trackWidth = 12;
    private double wheelBase = 6.5;
    private double lateralMultiplier = -1.2;

    public static double LATERAL_MULTIPLIER = 1.33;

    public static double CENTRIPETAL_CONVERSION = 0.1;

    public static double MASS = 26;

    @Override
    public void onInit(HardwareMap newHardwareMap, Telemetry newTelemetry) {

        robotInstance = RobotEx.getInstance();

        rightFrontMotor = new CachingDcMotorEX((DcMotorEx) newHardwareMap.get(DcMotor.class, "rightFrontMotor"), 1e-5);
        leftFrontMotor = new CachingDcMotorEX((DcMotorEx) newHardwareMap.get(DcMotor.class, "leftFrontMotor"), 1e-5);
        leftBackMotor = new CachingDcMotorEX((DcMotorEx) newHardwareMap.get(DcMotor.class, "leftBackMotor"), 1e-5);
        rightBackMotor = new CachingDcMotorEX((DcMotorEx) newHardwareMap.get(DcMotor.class, "rightBackMotor"), 1e-5);

        // todo: figure out the directions

        drivetrainMotors = new DcMotorEx[] {
                rightFrontMotor,
                leftFrontMotor,
                leftBackMotor,
                rightBackMotor
        };


        setRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        rightFrontMotor.setDirection(DcMotorEx.Direction.FORWARD);
        leftFrontMotor.setDirection(DcMotorEx.Direction.REVERSE);
        leftBackMotor.setDirection(DcMotorEx.Direction.REVERSE);
        rightBackMotor.setDirection(DcMotorEx.Direction.FORWARD);


        drivetrainMotorGroup = new MotorGroup<>(
                rightFrontMotor,
                leftFrontMotor,
                leftBackMotor,
                rightBackMotor
        );

        rightBackPower = 0;
        leftBackPower = 0;
        leftFrontPower = 0;
        rightFrontPower = 0;


        telemetry = newTelemetry;
    }

    public void enableAntiTip() {
        enableAntiTip = true;
    }

    public void disableAntiTip() {
        enableAntiTip = false;
    }

    public void enableHeadingRetention() {
        enableHeadingRetention = true;
    }

    public void disableHeadingRetention() {
        enableHeadingRetention = false;
    }

    @Override
    public void onOpmodeStarted() {
        robotInstance = RobotEx.getInstance();
        theLocalizer = robotInstance.theLocalizer;
    }

    @Override
    public void onCyclePassed() {

        headingPID.updateCoefficients(Drivetrain.kP, Drivetrain.kI, Drivetrain.kD, 0);

        rightBackMotor.setPower(rightBackPower);
        rightFrontMotor.setPower(rightFrontPower);
        leftBackMotor.setPower(leftBackPower);
        leftFrontMotor.setPower(leftFrontPower);

        lastRightBackPower = rightBackPower;
        lastLeftBackPower= leftBackPower;
        lastLeftFrontPower = leftFrontPower;
        lastRightFrontPower = rightFrontPower;

        rightBackPower = 0;
        leftBackPower = 0;
        leftFrontPower = 0;
        rightFrontPower = 0;
    }

    public void robotCentricDriveFromGamepad(double leftJoystickY, double leftJoystickX, double rightJoystickX) {

        leftJoystickX *= LATERAL_MULTIPLIER;

        double multiple = RobotEx.getInstance().getPowerMultiple();

        leftJoystickY = MathHelper.clamp(leftJoystickY * multiple, -1, 1);
        leftJoystickX = MathHelper.clamp(leftJoystickX * multiple, -1, 1);
        rightJoystickX = MathHelper.clamp(rightJoystickX * multiple, -1, 1);

        double denominator = Math.max(Math.abs(leftJoystickY) + Math.abs(leftJoystickX) + Math.abs(rightJoystickX), 1);
        this.leftFrontPower += (leftJoystickY + leftJoystickX + rightJoystickX) / denominator;
        this.leftBackPower += (leftJoystickY - leftJoystickX + rightJoystickX) / denominator;
        this.rightFrontPower += (leftJoystickY - leftJoystickX - rightJoystickX) / denominator;
        this.rightBackPower += (leftJoystickY + leftJoystickX - rightJoystickX) / denominator;

        this.lastX = leftJoystickX;
        this.lastY = leftJoystickY;
        this.lastRot = rightJoystickX;
    }

    public void fieldCentricDriveFromGamepad(double leftJoystickY, double leftJoystickX, double rightJoystickX) {
        double currentRobotOrientation = robotInstance.theLocalizer.currentPose.getHeading() - Math.PI / 2;

        this.robotCentricDriveFromGamepad(
                Math.sin(-currentRobotOrientation) * leftJoystickX + Math.cos(-currentRobotOrientation) * leftJoystickY,
                (Math.cos(-currentRobotOrientation) * leftJoystickX - Math.sin(-currentRobotOrientation) * leftJoystickY),
                rightJoystickX
        );
    }

    public void setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior newZeroPowerBehavior) {
        for (DcMotorEx drivetrainMotor : this.drivetrainMotors) {
            drivetrainMotor.setZeroPowerBehavior(newZeroPowerBehavior);
        }
    }

    public void setRunMode(DcMotor.RunMode newRunMode) {
        for (DcMotorEx drivetrainMotor : this.drivetrainMotors) {
            drivetrainMotor.setMode(newRunMode);
        }
    }

    public DcMotorEx[] getDrivetrainMotors() {
        return this.drivetrainMotors;
    }

    public MotorGroup<DcMotorEx> getDrivetrainMotorGroup() {
        return this.drivetrainMotorGroup;
    }

    public DcMotor.ZeroPowerBehavior getZeroPowerBehavior() {
        return this.leftBackMotor.getZeroPowerBehavior();
    }


    public int[] getCWMotorTicks() {
        return new int[] {
                this.rightFrontMotor.getCurrentPosition(),
                this.leftFrontMotor.getCurrentPosition(),
                this.leftBackMotor.getCurrentPosition(),
                this.rightBackMotor.getCurrentPosition()
        };
    }

    public MecanumWheelState getMotorTicks() {
        return new MecanumWheelState(
                rightFrontMotor.getCurrentPosition(),
                leftFrontMotor.getCurrentPosition(),
                leftBackMotor.getCurrentPosition(),
                rightBackMotor.getCurrentPosition()
        );
    }

    public MecanumWheelState getMotorVelocity() {
        return new MecanumWheelState(
                rightFrontMotor.getVelocity(),
                leftFrontMotor.getVelocity(),
                leftBackMotor.getVelocity(),
                rightBackMotor.getVelocity()
        );
    }

    public double getCentripetalCorrection() {
        Pose[] recentPoseAverage = theLocalizer.centripetalPoseBuffer.getAveragePoses();

        double radius = MathHelper.findRadiusFromPoints(recentPoseAverage);

        double velocityMagnitude = theLocalizer.getVelocity().magnitude();
        double velocity_mpers = MathHelper.inchesPerSecondToMetersPerSecond(velocityMagnitude);

        double centripetalForce = MASS * velocity_mpers * velocity_mpers / radius;
        return centripetalForce * CENTRIPETAL_CONVERSION;
    }

    public void setWeightedDrivePower(double weight) {
        this.weight = weight;
    }


    public double getTrackWidth() {
        return trackWidth;
    }

    public double getWheelBase() {
        return wheelBase;
    }

    public double getLateralMultiplier() {
        return lateralMultiplier;
    }
}

