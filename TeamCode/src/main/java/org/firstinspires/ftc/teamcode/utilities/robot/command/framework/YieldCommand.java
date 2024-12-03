package org.firstinspires.ftc.teamcode.utilities.robot.command.framework;

public class YieldCommand extends DelayCommand {

    private long theDuration;
    private long theStartTime;

    private AnonymousIsFinishedFunction theExhaustFunction;


    public YieldCommand(long aDuration) {
        super(0);

        theDuration = aDuration;
    }

    public YieldCommand(long aDuration, AnonymousIsFinishedFunction aFunction) {
        super(0);

        theDuration = aDuration;

        theExhaustFunction = aFunction;
    }


    @Override
    public void initialize() {
        theStartTime = System.currentTimeMillis();
    }

    @Override
    public void update() {

    }

    @Override
    public boolean isFinished() {

        boolean overTimeAllotted = System.currentTimeMillis() - theStartTime >= theDuration;

        if (theExhaustFunction != null) {
            System.out.println(theExhaustFunction.isFinished());
            return overTimeAllotted || theExhaustFunction.isFinished();
        }

        return overTimeAllotted;
    }

    @Override
    public void onFinish() {

    }
}
