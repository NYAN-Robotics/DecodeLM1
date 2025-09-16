package org.firstinspires.ftc.teamcode.utilities.robot.command.framework.commandtypes;

import org.firstinspires.ftc.teamcode.utilities.robot.command.framework.AnonymousIsFinishedFunction;

public class YieldCommand extends CommandBase {

    private long theDuration;
    private long theStartTime;

    private AnonymousIsFinishedFunction theExhaustFunction;


    public YieldCommand(long aDuration) {
        theDuration = aDuration;
    }

    public YieldCommand(AnonymousIsFinishedFunction aFunction) {
        theExhaustFunction = aFunction;
        theDuration = Long.MAX_VALUE;
    }

    public YieldCommand(long aDuration, AnonymousIsFinishedFunction aFunction) {
        theDuration = aDuration;
        theExhaustFunction = aFunction;
    }


    @Override
    public void onSchedule() {

    }

    @Override
    public boolean readyToExecute() {
        return true;
    }

    @Override
    public void initialize() {
        theStartTime = System.currentTimeMillis();
        // System.out.print("Initialize called" + theStartTime);
    }

    @Override
    public void update() {

    }

    @Override
    public boolean isFinished() {

        boolean overTimeAllotted = System.currentTimeMillis() - theStartTime >= theDuration;

        // System.out.println("Current Time: " + System.currentTimeMillis() + " Start Time: " + theStartTime);
        if (theExhaustFunction != null) {
            // System.out.println(theExhaustFunction.isFinished());
            return overTimeAllotted || theExhaustFunction.isFinished();
        }

        return overTimeAllotted;
    }

    @Override
    public void onFinish() {

    }
}
