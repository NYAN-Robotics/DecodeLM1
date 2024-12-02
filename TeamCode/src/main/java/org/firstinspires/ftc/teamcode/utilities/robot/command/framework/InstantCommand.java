package org.firstinspires.ftc.teamcode.utilities.robot.command.framework;

public abstract class InstantCommand extends CommandBase {

    @Override
    public boolean readyToExecute() {
        return true;
    }

    @Override
    public boolean isFinished() {
        return true;
    }

}
