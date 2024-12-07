package org.firstinspires.ftc.teamcode.utilities.robot.command.framework.commandtypes;


public class SequentialCommandGroup extends CommandBase {

    private final CommandBase[] theCommands;
    private int theCommandIndex = 0;
    private CommandBase theCurrentCommand;

    public SequentialCommandGroup(CommandBase... aCommandGroup) {
        this.theCommands = aCommandGroup;

        theCurrentCommand = theCommands[theCommandIndex];
    }

    @Override
    public void onSchedule() {
        for (CommandBase command : theCommands) {
            command.onSchedule();
        }
    }

    @Override
    public boolean readyToExecute() {
        return theCurrentCommand.readyToExecute();
    }

    @Override
    public void initialize() {
        theCurrentCommand.initialize();
    }

    @Override
    public void update() {

        if (!theCurrentCommand.readyToExecute()) {
            return;
        }

        theCurrentCommand.update();

        if (theCurrentCommand.isFinished()) {
            theCurrentCommand.onFinish();

            theCommandIndex++;

            if (theCommandIndex == theCommands.length) {
                return;
            }

            theCurrentCommand = theCommands[theCommandIndex];
            theCurrentCommand.initialize();
        }
    }

    @Override
    public boolean isFinished() {
        return theCommandIndex >= theCommands.length;
    }

    @Override
    public void onFinish() {
    }
}
