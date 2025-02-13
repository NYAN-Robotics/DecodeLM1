package org.firstinspires.ftc.teamcode.utilities.robot.command.framework.commandtypes;

import java.util.ArrayList;
import java.util.List;

public class DeadlineCommand extends CommandBase {

    private final CommandBase deadlineCommand;
    private final List<CommandBase> commands;

    public DeadlineCommand(CommandBase deadlineCommand, CommandBase... commands) {
        this.deadlineCommand = deadlineCommand;
        this.commands = new ArrayList<>();

        for (CommandBase command : commands) {
            this.commands.add(command);
        }
    }

    @Override
    public void onSchedule() {
        deadlineCommand.onSchedule();
        for (CommandBase command : commands) {
            command.onSchedule();
        }
    }

    @Override
    public boolean readyToExecute() {
        return deadlineCommand.readyToExecute();
    }

    @Override
    public void initialize() {
        deadlineCommand.initialize();
        for (CommandBase command : commands) {
            command.initialize();
        }
    }

    @Override
    public void update() {
        deadlineCommand.update();
        for (CommandBase command : commands) {
            command.update();
        }
    }

    @Override
    public boolean isFinished() {
        if (deadlineCommand.isFinished()) {
            return true;
        }
        for (CommandBase command : commands) {
            if (!command.isFinished()) {
                return false;
            }
        }
        return true;
    }

    @Override
    public void onFinish() {
        deadlineCommand.onFinish();
        for (CommandBase command : commands) {
            command.onFinish();
        }
    }
}