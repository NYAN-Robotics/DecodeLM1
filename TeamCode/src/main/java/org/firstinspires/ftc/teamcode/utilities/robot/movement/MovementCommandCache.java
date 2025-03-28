package org.firstinspires.ftc.teamcode.utilities.robot.movement;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.utilities.controltheory.motionprofiler.MotionProfile;
import org.firstinspires.ftc.teamcode.utilities.math.MathHelper;
import org.firstinspires.ftc.teamcode.utilities.math.linearalgebra.Pose;
import org.firstinspires.ftc.teamcode.utilities.physics.states.KinematicState;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;

public class MovementCommandCache {

    private final double CACHE_INCREMENT = 1/2000.0;
    private ArrayList<MovementStateCommand> theStateCache;

    MovementConstants theMovementConstants;

    MotionProfile theMotionProfile;

    ElapsedTime theProfileTimer;

    Pose theStartPose;
    Pose theEndPose;

    Pose theDeltaPose;

    public double theCurrentTime;

    double theDisplacement;
    double theDirection;

    double theSine;
    double theCosine;

    double theDuration;

    public MovementCommandCache(Pose aInitialPose, Pose aFinalPose, MovementConstants aConstants) {
        theStartPose = aInitialPose;
        theEndPose = aFinalPose;

        theDeltaPose = (new Pose(aFinalPose));
        theDeltaPose.subtract(aInitialPose);

        theDisplacement = theDeltaPose.magnitude();

        theDirection = Math.atan2(theDeltaPose.getY(), theDeltaPose.getX());
        theSine = Math.sin(theDirection);
        theCosine = Math.cos(theDirection);

        theMotionProfile = new MotionProfile(0, theDisplacement, aConstants.velocityMax, aConstants.accelerationMax);
        theDuration = theMotionProfile.getDuration();

        theProfileTimer = new ElapsedTime();
        theStateCache = new ArrayList<>((int) (theDuration / CACHE_INCREMENT + 1));

        theCurrentTime = 0;
        theMovementConstants = aConstants;

        cacheStates();
    }

    public void start() {
        theProfileTimer.reset();

        update();
    }

    public void cacheStates() {
        theStateCache.clear();

        for (double time = 0; time <= theDuration; time += CACHE_INCREMENT) {
            theStateCache.add(getKeyFromTime(time), getMovementStateCommand(time));
            // System.out.println("Time: " + time + " Duration: " + theDuration + "Key: " + getKeyFromTime(time) + "max: " + theDuration/CACHE_INCREMENT);
        }
    }

    public MovementStateCommand getTargetState() {

        // snap time to the nearest 0.01 number
        int key = getKeyFromTime(theCurrentTime); // Math.round(theCurrentTime / CACHE_INCREMENT) * CACHE_INCREMENT;;

        // System.out.println("Key: "+ key);
        return theStateCache.get(key);

        /*
        if (theStateCache.containsKey(key)) {
            return theStateCache.get(key);
        } else {
            System.out.println(key + " cache miss");

            MovementStateCommand targetState = getMovementStateCommand(theCurrentTime);

            theStateCache.put(theCurrentTime, targetState);
            return targetState;
        }

         */
    }

    public int getKeyFromTime(double time) {
        if (theDuration==0){
            return 0;
        }
        return (int) MathHelper.clamp(time / CACHE_INCREMENT, 0, theDuration/CACHE_INCREMENT - 1);
    }

    private MovementStateCommand getMovementStateCommand(double aTime) {
        KinematicState targetState = getKinematicState(aTime);

        double feedforward = theMovementConstants.kV * targetState.getVelocity() + theMovementConstants.kA * targetState.getAcceleration();

        return new MovementStateCommand(
                targetState.getPose(),
                feedforward * theSine,
                feedforward * theCosine
        );
    }

    private KinematicState getKinematicState(double aTime) {
        Pose targetPose = getTargetPose(aTime);
        double velocity = theMotionProfile.getVelocityFromTime(aTime);
        double acceleration = theMotionProfile.getAccelerationFromTime(aTime);

        return new KinematicState(targetPose, velocity, acceleration);
    }

    private Pose getTargetPose(double aTime) {
        double targetDisplacement = theMotionProfile.getPositionFromTime(aTime);
        double xTarget = theCosine * targetDisplacement + theStartPose.getX();
        double yTarget = theSine * targetDisplacement + theStartPose.getY();


        double headingTarget = MathHelper.lerp(
                theStartPose.getHeading(),
                theEndPose.getHeading(),
                Math.min(aTime + 0.05, theDuration) / theDuration
        );

        return new Pose(
                xTarget,
                yTarget,
                headingTarget
        );
    }

    public void update() {
        theCurrentTime = theProfileTimer.seconds();
    }

    public MotionProfile getTheMotionProfile() {
        return theMotionProfile;
    }

}
