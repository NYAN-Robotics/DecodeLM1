package org.firstinspires.ftc.teamcode.utilities.robot.command.framework;

public class YieldCommand extends DelayCommand {

    private long theDuration;
    private long theStartTime;

    public YieldCommand(long aDuration) {
        super(aDuration);
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
        return System.currentTimeMillis() - theStartTime >= theDuration;
    }

    @Override
    public void onFinish() {

    }
}
