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


        while (theCommandIndex < theCommands.length && theCurrentCommand.readyToExecute()) {
            // Update the current command
            theCurrentCommand.update();

            // If the current command is finished, move to the next one
            if (theCurrentCommand.isFinished()) {
                // Clean up the current command
                theCurrentCommand.onFinish();

                // Move to the next command
                theCommandIndex++;

                // If we've completed all commands, exit
                if (theCommandIndex >= theCommands.length) {
                    return;
                }

                // Set up the next command
                theCurrentCommand = theCommands[theCommandIndex];
                theCurrentCommand.initialize();
            } else {
                // Current command is not finished, so stop processing for this update cycle
                return;
            }
        }
        /*


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

         */


    }

    @Override
    public boolean isFinished() {
        return theCommandIndex >= theCommands.length;
    }

    @Override
    public void onFinish() {
    }
}
