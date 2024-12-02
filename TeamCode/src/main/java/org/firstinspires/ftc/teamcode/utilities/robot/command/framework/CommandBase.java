package org.firstinspires.ftc.teamcode.utilities.robot.command.framework;

public abstract class CommandBase {
    public abstract void onSchedule();
    public abstract boolean readyToExecute(); // New function
    public abstract void initialize();
    public abstract void update();
    public abstract boolean isFinished();
    public abstract void onFinish();
}
