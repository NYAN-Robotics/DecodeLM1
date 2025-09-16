package org.firstinspires.ftc.teamcode.utilities.robot.command.framework;

import org.firstinspires.ftc.teamcode.utilities.robot.command.framework.commandtypes.CommandBase;

import java.util.ArrayList;
import java.util.LinkedList;
import java.util.Queue;

public class CommandScheduler {

    private static final CommandScheduler INSTANCE = new CommandScheduler();

    private final ArrayList<CommandBase> theScheduledCommands = new ArrayList<>();
    private final Queue<CommandBase> theExecutingCommands = new LinkedList<>();

    private CommandScheduler() {}

    public static CommandScheduler getInstance() {
        return INSTANCE;
    }

    public void scheduleCommand(CommandBase aCommand) {
        aCommand.onSchedule();
        theScheduledCommands.add(aCommand);
    }

    public void clearCommands() {
        theScheduledCommands.clear();
        theExecutingCommands.clear();
    }

    public void update() {

        ArrayList<CommandBase> commandsToRemove = new ArrayList<>();

        for (CommandBase command : theScheduledCommands) {
            if (command.readyToExecute()) {
                command.initialize();
                theExecutingCommands.add(command);
                commandsToRemove.add(command);
            }
        }

        for (CommandBase command : commandsToRemove) {
            theScheduledCommands.remove(command);
        }

        executeCommands();
    }

    private void executeCommands() {
        ArrayList<CommandBase> commandsToRemove = new ArrayList<>();

        for (CommandBase command : theExecutingCommands) {
            command.update();
            if (command.isFinished()) {
                command.onFinish();
                commandsToRemove.add(command);
            }
        }

        for (CommandBase command : commandsToRemove) {
            theExecutingCommands.remove(command);
        }

    }


}