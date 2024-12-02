package org.firstinspires.ftc.teamcode.utilities.robot.command.framework;

import java.util.ArrayList;
import java.util.LinkedList;
import java.util.Queue;

public class CommandScheduler {

    private static final CommandScheduler INSTANCE = new CommandScheduler();

    private final ArrayList<CommandBase> scheduledCommands = new ArrayList<>();
    private final Queue<CommandBase> executingCommands = new LinkedList<>();

    private CommandScheduler() {} // Private constructor to enforce singleton

    public static CommandScheduler getInstance() {
        return INSTANCE;
    }

    public void scheduleCommand(CommandBase command) {
        command.onSchedule();
        scheduledCommands.add(command);
    }

    public void update() {

        ArrayList<CommandBase> commandsToRemove = new ArrayList<>();

        long currentTime = System.currentTimeMillis();

        // Move commands with exhausted delays to executing queue

        for (CommandBase command : scheduledCommands) {
            if (command.readyToExecute()) {
                command.initialize();
                executingCommands.add(command);
                commandsToRemove.add(command);
            }
        }

        for (CommandBase command : commandsToRemove) {
            scheduledCommands.remove(command);
        }
        // Execute commands in the executing queue
        executeCommands();
    }

    private void executeCommands() {
        ArrayList<CommandBase> commandsToRemove = new ArrayList<>();

        for (CommandBase command : executingCommands) {
            command.update();
            if (command.isFinished()) {
                command.onFinish();
                commandsToRemove.add(command);
            }
        }

        for (CommandBase command : commandsToRemove) {
            executingCommands.remove(command);
        }

    }


}