package org.firstinspires.ftc.teamcode.utilities.robot.command.framework.commandtypes;


import org.firstinspires.ftc.teamcode.utilities.robot.command.framework.AnonymousRunFunction;

public class LoopCommand extends CommandBase {

    private AnonymousRunFunction executable;

    public LoopCommand(AnonymousRunFunction aExecutable) {
        executable = aExecutable;
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

    }

    @Override
    public void update() {
        executable.execute();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void onFinish() {
        executable.execute();
    }
}
