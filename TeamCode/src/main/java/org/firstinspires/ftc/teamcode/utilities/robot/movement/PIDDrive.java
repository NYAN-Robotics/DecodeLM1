package org.firstinspires.ftc.teamcode.utilities.robot.movement;

import static org.firstinspires.ftc.teamcode.utilities.robot.DriveConstants.MAX_CORRECTION_TIME;
import static org.firstinspires.ftc.teamcode.utilities.robot.DriveConstants.THRESHOLD;

import androidx.core.math.MathUtils;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.utilities.controltheory.feedback.GeneralPIDController;
import org.firstinspires.ftc.teamcode.utilities.controltheory.motionprofiler.MotionProfile;
import org.firstinspires.ftc.teamcode.utilities.math.AngleHelper;
import org.firstinspires.ftc.teamcode.utilities.math.MathHelper;
import org.firstinspires.ftc.teamcode.utilities.math.linearalgebra.Pose;
import org.firstinspires.ftc.teamcode.utilities.robot.DriveConstants;
import org.firstinspires.ftc.teamcode.utilities.robot.RobotEx;
import org.firstinspires.ftc.teamcode.utilities.robot.extensions.MotorGroup;

@Config
public class PIDDrive {
    //  GeneralPIDController xController = new GeneralPIDController(0.3, 0, 0, 0);
    // GeneralPIDController yController = new GeneralPIDController(0.3, 0, 0, 0);

    GeneralPIDController xController = new GeneralPIDController(0.2, 0, 1, 0);
    GeneralPIDController yController = new GeneralPIDController(0.2, 0, 1, 0);
    GeneralPIDController headingController = new GeneralPIDController(1.5, 0, 0, 0);

    public static double vMax = 50;
    public static double aMax = 40;

    public static double kA = 0.0035;
    public static double kV = 0.013;

    public static Pose threshold = new Pose(0.25, 0.25, Math.toRadians(2));
    public static double thresholdTime = 1;

    RobotEx robot;

    Telemetry telemetry;
    LinearOpMode opmode;

    public PIDDrive(RobotEx newRobot, LinearOpMode newOpmode, Telemetry newTelemetry) {
        robot = newRobot;
        opmode = newOpmode;
        telemetry = newTelemetry;
    }


    public void gotoPoint(Pose point) {
        this.gotoPoint(
                point,
                new MovementConstants(vMax, aMax, MAX_CORRECTION_TIME, kV, kA)
        );
    }

    public void gotoPoint(Pose point, double correctionTime) {
        this.gotoPoint(
                point,
                new MovementConstants(vMax, aMax, correctionTime, kV, kA)
        );
    }

    public void gotoPoint(Pose point, MovementConstants constants) {
        Pose currentPose = robot.odometry.getPose();
        Pose currentVelocity = new Pose();

        Pose startPosition = new Pose(currentPose.getX(), currentPose.getY(), currentPose.getHeading());
        Pose error = new Pose(
                point.getX() - currentPose.getX(),
                point.getY() - currentPose.getY(),
                AngleHelper.normDelta(point.getHeading()) - AngleHelper.normDelta(currentPose.getHeading())
        );

        double displacement = Math.sqrt(error.getX() * error.getX() + error.getY() * error.getY());

        MotionProfile motion = new MotionProfile(
                0, displacement, constants.velocityMax, constants.accelerationMax
        );

        double angle = Math.atan2(error.getY(), error.getX());
        double sineTerm = Math.sin(angle);
        double cosineTerm = Math.cos(angle);

        double duration = motion.getDuration();
        double currentProfileTime = 0;

        boolean inPosition = false;

        ElapsedTime inPositionTime = new ElapsedTime();
        ElapsedTime profileTime = new ElapsedTime();

        while (!robot.stopRequested) {

            currentPose = robot.odometry.getPose();
            currentProfileTime = profileTime.seconds();

            double targetDisplacement = motion.getPositionFromTime(currentProfileTime);
            double xTarget = cosineTerm * targetDisplacement + startPosition.getX();
            double yTarget = sineTerm * targetDisplacement + startPosition.getY();


            double headingTarget = MathHelper.lerp(startPosition.getHeading(), point.getHeading(), Math.min(currentProfileTime + 0.05, duration) / duration);

            error = new Pose(
                    xTarget - currentPose.getX(),
                    yTarget - currentPose.getY(),
                    AngleHelper.normDelta(headingTarget) - AngleHelper.normDelta(currentPose.getHeading())
            );


            if (Math.abs(error.getHeading()) > Math.PI) {
                error.setHeading(
                        AngleHelper.norm(headingTarget) - AngleHelper.norm(currentPose.getHeading())
                );
            }

            double currentAcceleration = motion.getAccelerationFromTime(currentProfileTime);
            double currentVelocityAmount = motion.getVelocityFromTime(currentProfileTime);

            double feedforward = currentAcceleration * kA + currentVelocityAmount * kV;

            double feedforwardX = feedforward * sineTerm;
            double feedforwardY = feedforward * cosineTerm;

            double feedbackX = xController.getOutputFromError(error.getX());
            double feedbackY = yController.getOutputFromError(error.getY());

            robot.drivetrain.fieldCentricDriveFromGamepad(
                    feedforwardX + feedbackY,
                    feedforwardY + feedbackX,
                    -MathUtils.clamp(headingController.getOutputFromError(
                            error.getHeading()
                    ), -0.75, 0.75)
            );

            // telemetry.addData("Profile time: ", currentProfileTime);
            // telemetry.addData("Motion time: ", duration);
            // telemetry.addData("Error h: ", error.getHeading());

            telemetry.addData("Target Velocity X: ", currentVelocityAmount * sineTerm);
            telemetry.addData("Target Velocity Y: ", currentVelocityAmount * cosineTerm);

            /*
            telemetry.addData("Error y: ", error.getY());
            telemetry.addData("feedback x: ", feedbackX);
            telemetry.addData("feedback y: ", feedbackY);

             */

            error.map(Math::abs);
            currentVelocity = new Pose(currentVelocity).map(Math::abs);

            if (error.lessThan(threshold) && currentProfileTime > duration) {
                if (inPosition) {
                    if (inPositionTime.seconds() >= thresholdTime || currentVelocity.lessThan(threshold)) {
                        break;
                    }
                } else {
                    inPosition = true;
                    inPositionTime.reset();
                }
            } else if (currentProfileTime > duration + constants.maxCorrectionTime) {
                break;
            } else {
                inPosition = false;
            }

            robot.update();
        }

        robot.drivetrain.fieldCentricDriveFromGamepad(
                0,
                0,
                0
        );

        robot.update();
    }

    public void gotoPoint(MovementCommandCache movementCommandCache) {

        Pose currentPose;
        Pose currentVelocity;

        Pose error = new Pose();

        ElapsedTime inPositionTime = new ElapsedTime();
        boolean inPosition = false;

        movementCommandCache.start();

        while (!robot.stopRequested) {
            movementCommandCache.update();

            MovementStateCommand targetState = movementCommandCache.getTargetState();

            currentPose = robot.odometry.getPose();
            currentVelocity = robot.odometry.getVelocity();

            error.setX(targetState.getPose().getX() - currentPose.getX());
            error.setY(targetState.getPose().getY() - currentPose.getY());
            error.setHeading(
                    AngleHelper.normDelta(targetState.getPose().getHeading()) - AngleHelper.normDelta(currentPose.getHeading())
            );

            telemetry.addData("Angle Error initial: ", error.getHeading());
            if (Math.abs(error.getHeading()) > Math.PI) {
                error.setHeading(
                        AngleHelper.norm(targetState.getPose().getHeading()) - AngleHelper.norm(currentPose.getHeading())
                );
            }

            double feedbackX = xController.getOutputFromError(error.getX());
            double feedbackY = yController.getOutputFromError(error.getY());

            robot.drivetrain.fieldCentricDriveFromGamepad(
                    targetState.feedforwardX + feedbackY,
                    targetState.feedforwardY + feedbackX,
                    -MathUtils.clamp(headingController.getOutputFromError(
                            error.getHeading()
                    ), -0.75, 0.75)
            );

            error.abs();
            currentVelocity.abs();

            /*
            telemetry.addData("Current Time: ", movementCommand.currentTime);
            telemetry.addData("Snap Time: ", Math.round(movementCommand.currentTime / (1/100.0)) * (1/100.0));
            telemetry.addData("X: ", targetState.feedforwardX + feedbackY);
            telemetry.addData("Y: ", targetState.feedforwardY + feedbackX);
            telemetry.addData("Feedforward: ", targetState.feedforwardX);
            telemetry.addData("bool: ", movementCommand.currentTime > movementCommand.duration + movementCommand.constants.maxCorrectionTime);
             */

            if (error.lessThan(THRESHOLD) && movementCommandCache.theCurrentTime > movementCommandCache.theDuration) {
                if (inPosition) {
                    if (inPositionTime.seconds() >= movementCommandCache.theMovementConstants.maxCorrectionTime || currentVelocity.lessThan(THRESHOLD)) {
                        break;
                    }
                } else {
                    inPosition = true;
                    inPositionTime.reset();
                }
            } else if (movementCommandCache.theCurrentTime > movementCommandCache.theDuration + movementCommandCache.theMovementConstants.maxCorrectionTime) {
                break;
            } else {
                inPosition = false;
            }



            robot.update();

        }
    }


    public void turnToAngle(double angle, MovementConstants constants) {
        angle = AngleHelper.normDelta(angle);

        double currentIMUPosition = robot.odometry.getPose().getHeading();
        double turnError;

        MotionProfile turnProfile = new MotionProfile(currentIMUPosition, angle, DriveConstants.MAX_ANGULAR_VELOCITY, DriveConstants.MAX_ANGULAR_VELOCITY);

        ElapsedTime turnTimer = new ElapsedTime();
        ElapsedTime elapsedTurnTime = new ElapsedTime();

        boolean atTarget = false;
        double atTargetStartTime = -1;

        while (!atTarget && elapsedTurnTime.seconds() < turnProfile.getDuration() + constants.maxCorrectionTime) {

            double currentTargetAngle = turnProfile.getPositionFromTime(elapsedTurnTime.seconds());
            turnError = currentTargetAngle - currentIMUPosition;

            if (Math.abs(turnError) > Math.PI) {
                if (angle < 0) {
                    angle = AngleHelper.norm(angle);
                    turnError = angle - currentIMUPosition;
                } else if (angle > 0) {
                    currentIMUPosition = AngleHelper.norm(currentIMUPosition);
                    turnError = angle - currentIMUPosition;
                }
            }

            double output = robot.drivetrain.profiledTurningPID.getOutputFromError(
                    turnError
            );


            robot.drivetrain.robotCentricDriveFromGamepad(
                    0,
                    0,
                    -Math.min(Math.max(output, -1), 1) + Math.signum(output)
            );

            currentIMUPosition = robot.odometry.getPose().getHeading();

            if (telemetry != null) {
                telemetry.addData("Target Angle: ", currentTargetAngle);
                telemetry.addData("Turn Angle: ", turnError);
                telemetry.addData("current angle: ", currentIMUPosition);
                telemetry.update();
            }

            this.robot.update();
        }

        MotorGroup<DcMotorEx> robotDrivetrain = robot.drivetrain.getDrivetrainMotorGroup();

        DcMotor.ZeroPowerBehavior currentZeroPowerBehavior = robot.drivetrain.getZeroPowerBehavior();

        if (currentZeroPowerBehavior != DcMotor.ZeroPowerBehavior.BRAKE) {
            robotDrivetrain.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            robotDrivetrain.setPower(0);
            robot.pause(0.1);
            robotDrivetrain.setZeroPowerBehavior(currentZeroPowerBehavior);
        }


    }

    public void turnToAngle(double angle) {
        turnToAngle(angle, new MovementConstants(0, 0, MAX_CORRECTION_TIME, 0, 0));
    }
}
