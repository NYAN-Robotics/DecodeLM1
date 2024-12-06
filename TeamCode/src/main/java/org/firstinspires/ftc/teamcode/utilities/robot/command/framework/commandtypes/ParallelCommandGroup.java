package org.firstinspires.ftc.teamcode.utilities.robot.command.framework.commandtypes;

public class ParallelCommandGroup extends InstantCommand {

        private final CommandBase[] commands;

        private boolean[] initialized;
        private boolean[] finished;

        public ParallelCommandGroup(CommandBase... commands) {
            this.commands = commands;

            initialized = new boolean[commands.length];
            finished = new boolean[commands.length];
        }

        @Override
        public void onSchedule() {
            for (CommandBase command : commands) {
                command.onSchedule();
            }
        }

        @Override
        public void initialize() {
        }

        @Override
        public void update() {
            for (int i = 0; i < commands.length; i++) {
                CommandBase command = commands[i];

                if (command.readyToExecute()) {
                    if (!initialized[i]) {
                        command.initialize();
                        initialized[i] = true;
                    }

                    if (!command.isFinished()) {
                        command.update();
                    } else {
                        if (!finished[i]) {
                            command.onFinish();
                            finished[i] = true;
                        }
                    }
                }
            }
        }

        @Override
        public boolean isFinished() {
            for (CommandBase command : commands) {
                if (!command.readyToExecute() || !command.isFinished()) {
                    return false;
                }
            }
            return true;
        }

        @Override
        public void onFinish() {
        }
}