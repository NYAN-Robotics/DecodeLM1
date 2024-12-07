package org.firstinspires.ftc.teamcode.utilities.robot.command.framework.commandtypes;

public class ParallelCommandGroup extends InstantCommand {

        private final CommandBase[] theCommands;

        private boolean[] theInitializedCommands;
        private boolean[] theFinishedCommands;

        public ParallelCommandGroup(CommandBase... aCommandGroup) {
            this.theCommands = aCommandGroup;

            theInitializedCommands = new boolean[aCommandGroup.length];
            theFinishedCommands = new boolean[aCommandGroup.length];
        }

        @Override
        public void onSchedule() {
            for (CommandBase command : theCommands) {
                command.onSchedule();
            }
        }

        @Override
        public void initialize() {
        }

        @Override
        public void update() {
            for (int i = 0; i < theCommands.length; i++) {
                CommandBase command = theCommands[i];

                if (command.readyToExecute()) {
                    if (!theInitializedCommands[i]) {
                        command.initialize();
                        theInitializedCommands[i] = true;
                    }

                    if (!command.isFinished()) {
                        command.update();
                    } else {
                        if (!theFinishedCommands[i]) {
                            command.onFinish();
                            theFinishedCommands[i] = true;
                        }
                    }
                }
            }
        }

        @Override
        public boolean isFinished() {
            for (CommandBase command : theCommands) {
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