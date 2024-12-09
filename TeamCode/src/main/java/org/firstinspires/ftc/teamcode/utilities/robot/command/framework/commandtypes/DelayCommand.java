package org.firstinspires.ftc.teamcode.utilities.robot.command.framework.commandtypes;

public abstract class DelayCommand extends CommandBase {

    private long theTargetExecutionTime;
    private long theDelay;

    public DelayCommand(long aDelay) {
        theDelay = aDelay;
    }

    public DelayCommand(long aDuration, Object o) {
        super();
    }

    @Override
    public void onSchedule() {
        theTargetExecutionTime = System.currentTimeMillis() + theDelay;
    }

    @Override
    public boolean readyToExecute() {
        return System.currentTimeMillis() >= theTargetExecutionTime;
    }
}
