package org.firstinspires.ftc.teamcode.utilities.robot.command.framework;


public class OneTimeCommand extends CommandBase {

    private AnonymousInitializationCommand theInitializationCommand;

    public OneTimeCommand(AnonymousInitializationCommand aCommand) {
        theInitializationCommand = aCommand;
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
        theInitializationCommand.initialize();
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

    }
}
