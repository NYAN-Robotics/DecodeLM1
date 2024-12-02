package org.firstinspires.ftc.teamcode.utilities.robot.command.framework;

public class SequentialCommand extends CommandBase {

    private final CommandBase[] commands;
    private int currentCommandIndex = 0;

    public SequentialCommand(CommandBase... commands) {
        this.commands = commands;
    }

    @Override
    public void onSchedule() {
        commands[0].onSchedule();
    }

    @Override
    public boolean readyToExecute() {
        return commands[0].readyToExecute();
    }

    @Override
    public void initialize() {
        commands[0].initialize();
    }

    @Override
    public void update() {
        if (!commands[currentCommandIndex].readyToExecute()) {
            return;
        }

        commands[currentCommandIndex].update();
        if (commands[currentCommandIndex].isFinished()) {
            commands[currentCommandIndex].onFinish();
            currentCommandIndex++;
            if (currentCommandIndex < commands.length) {
                commands[currentCommandIndex].onSchedule();
            }
        }
    }

    @Override
    public boolean isFinished() {
        return currentCommandIndex >= commands.length;
    }

    @Override
    public void onFinish() {
    }
}
