package org.firstinspires.ftc.teamcode.utilities.robot.movement;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.utilities.controltheory.motionprofiler.MotionProfile;
import org.firstinspires.ftc.teamcode.utilities.math.MathHelper;
import org.firstinspires.ftc.teamcode.utilities.math.linearalgebra.Pose;
import org.firstinspires.ftc.teamcode.utilities.physics.states.KinematicState;

import java.util.HashMap;
import java.util.Map;

public class MovementCommand {

    private final double CACHE_INCREMENT = 1/100.0;
    private Map<Double, MovementStateCommand> stateCache;

    MovementConstants constants;

    MotionProfile motionProfile;

    ElapsedTime profileTimer;

    Pose startPose;
    Pose endPose;

    Pose deltaPose;

    double currentTime;

    double displacement;
    double direction;

    double sine;
    double cosine;

    double duration;

    public MovementCommand(Pose aInitialPose, Pose aFinalPose, MovementConstants aConstants) {
        startPose = aInitialPose;
        endPose = aFinalPose;

        deltaPose = (new Pose(aFinalPose));
        deltaPose.subtract(aInitialPose);

        displacement = deltaPose.magnitude();

        direction = Math.atan2(deltaPose.getY(), deltaPose.getX());
        sine = Math.sin(direction);
        cosine = Math.cos(direction);

        motionProfile = new MotionProfile(0, displacement, aConstants.velocityMax, aConstants.accelerationMax);

        profileTimer = new ElapsedTime();

        currentTime = 0;

        stateCache = new HashMap<>();

        constants = aConstants;
    }

    public void start() {
        profileTimer.reset();
    }

    public void cacheStates() {
        stateCache.clear();

        for (double time = 0; time <= duration; time += 1/CACHE_INCREMENT) {
            stateCache.put(time, getMovementStateCommand(time));
        }
    }

    public MovementStateCommand getTargetState() {

        // snap time to the nearest 0.01 number
        double key = Math.round(currentTime / CACHE_INCREMENT) * CACHE_INCREMENT;;

        if (stateCache.containsKey(key)) {
            return stateCache.get(key);
        } else {

            MovementStateCommand targetState = getMovementStateCommand(currentTime);

            stateCache.put(currentTime, targetState);
            return targetState;
        }
    }

    private MovementStateCommand getMovementStateCommand(double time) {
        KinematicState targetState = getKinematicState(time);

        double feedforward = constants.kV * targetState.getVelocity() + constants.kA * targetState.getAcceleration();

        return new MovementStateCommand(
                targetState.getPose(),
                feedforward * sine,
                feedforward * cosine
        );
    }

    private KinematicState getKinematicState(double time) {
        Pose targetPose = getTargetPose(time);
        double velocity = motionProfile.getVelocityFromTime(time);
        double acceleration = motionProfile.getAccelerationFromTime(time);

        return new KinematicState(targetPose, velocity, acceleration);
    }

    private Pose getTargetPose(double time) {
        double targetDisplacement = motionProfile.getPositionFromTime(time);
        double xTarget = cosine * targetDisplacement + startPose.getX();
        double yTarget = sine * targetDisplacement + startPose.getY();


        double headingTarget = MathHelper.lerp(
                startPose.getHeading(),
                endPose.getHeading(),
                Math.min(time + 0.05, duration) / duration
        );

        return new Pose(
                xTarget,
                yTarget,
                headingTarget
        );
    }

    public void update() {
        currentTime = profileTimer.seconds();
    }

}
