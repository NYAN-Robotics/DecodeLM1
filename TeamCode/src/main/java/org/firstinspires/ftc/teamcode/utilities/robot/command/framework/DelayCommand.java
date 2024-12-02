package org.firstinspires.ftc.teamcode.utilities.robot.command.framework;

public abstract class DelayCommand extends CommandBase {

    private long theTargetExecutionTime;
    private long theDelay;

    public DelayCommand(long aDelay) {
        theDelay = aDelay;
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
