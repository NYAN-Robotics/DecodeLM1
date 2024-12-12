package org.firstinspires.ftc.teamcode.utilities.robot.command.movement;

import static org.firstinspires.ftc.teamcode.utilities.robot.DriveConstants.THRESHOLD;

import androidx.core.math.MathUtils;

import org.firstinspires.ftc.teamcode.utilities.controltheory.motionprofiler.MotionProfile;
import org.firstinspires.ftc.teamcode.utilities.math.AngleHelper;
import org.firstinspires.ftc.teamcode.utilities.math.linearalgebra.Pose;
import org.firstinspires.ftc.teamcode.utilities.robot.RobotEx;
import org.firstinspires.ftc.teamcode.utilities.robot.command.framework.commandtypes.InstantCommand;
import org.firstinspires.ftc.teamcode.utilities.robot.movement.MovementCommandCache;
import org.firstinspires.ftc.teamcode.utilities.robot.movement.MovementConstants;
import org.firstinspires.ftc.teamcode.utilities.robot.movement.MovementStateCommand;

public class MovementCommand extends InstantCommand {

    RobotEx theRobot = RobotEx.getInstance();

    Pose theInitialPose;
    Pose theFinalPose;

    MovementConstants theConstants;

    Pose error;

    MovementCommandCache theCache;

    MotionProfile theMotionProfile;

    public MovementCommand(Pose aInitialPose, Pose aFinalPose, MovementConstants aConstants) {
        theInitialPose = aInitialPose;
        theFinalPose = aFinalPose;

        theConstants = aConstants;

        theCache = new MovementCommandCache(
                aInitialPose,
                aFinalPose,
                aConstants
        );

        theMotionProfile = theCache.getTheMotionProfile();
    }

    @Override
    public void onSchedule() {

    }

    @Override
    public void initialize() {
        theCache.start();
    }

    @Override
    public void update() {

        theCache.update();

        MovementStateCommand targetState = theCache.getTargetState();
        Pose targetPose = targetState.getPose();

        Pose currentPose = theRobot.theOpticalOdometry.getPose();
        Pose currentVelocity = theRobot.theOpticalOdometry.getVelocity();

        Pose error = new Pose(0, 0, 0);

        error.setX(targetState.getPose().getX() - currentPose.getX());
        error.setY(targetState.getPose().getY() - currentPose.getY());
        error.setHeading(
                AngleHelper.normDelta(targetState.getPose().getHeading()) - AngleHelper.normDelta(currentPose.getHeading())
        );

        if (Math.abs(error.getHeading()) > Math.PI) {
            error.setHeading(
                    AngleHelper.norm(targetState.getPose().getHeading()) - AngleHelper.norm(currentPose.getHeading())
            );
        }

        double feedbackX = theRobot.theDrivetrain.xController.getOutputFromError(error.getX());
        double feedbackY = theRobot.theDrivetrain.yController.getOutputFromError(error.getY());

        System.out.println("Feedforward x: " + targetState.getFeedforwardX() + "Feedforward y: " + targetState.getFeedforwardY());
        theRobot.theDrivetrain.fieldCentricDriveFromGamepad(
                targetState.getFeedforwardX() + feedbackY,
                targetState.getFeedforwardY() + feedbackX,
                -MathUtils.clamp(theRobot.theDrivetrain.headingController.getOutputFromError(
                        error.getHeading()
                ), -0.75, 0.75)
        );
    }

    @Override
    public boolean isFinished() {
        return theMotionProfile.getDuration() < theCache.theCurrentTime - theConstants.maxCorrectionTime;
    }

    @Override
    public void onFinish() {

    }
}
