package org.firstinspires.ftc.teamcode.utilities.robot.command.framework.commandtypes;


import org.firstinspires.ftc.teamcode.utilities.robot.command.framework.AnonymousRunFunction;

public class OneTimeCommand extends CommandBase {

    private AnonymousRunFunction executable;

    public OneTimeCommand(AnonymousRunFunction aExecutable) {
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

    }

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void onFinish() {
        executable.execute();
    }
}
