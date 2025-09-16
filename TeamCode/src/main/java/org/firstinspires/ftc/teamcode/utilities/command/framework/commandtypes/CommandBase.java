package org.firstinspires.ftc.teamcode.utilities.robot.command.framework.commandtypes;

public abstract class CommandBase {
    public abstract void onSchedule();
    public abstract boolean readyToExecute();
    public abstract void initialize();
    public abstract void update();
    public abstract boolean isFinished();
    public abstract void onFinish();
}
