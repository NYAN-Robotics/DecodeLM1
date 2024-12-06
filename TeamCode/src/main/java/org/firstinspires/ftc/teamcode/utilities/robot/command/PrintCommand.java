package org.firstinspires.ftc.teamcode.utilities.robot.command;

import org.firstinspires.ftc.teamcode.utilities.robot.command.framework.commandtypes.InstantCommand;

public class PrintCommand extends InstantCommand {

    private String message;


    public PrintCommand(String message) {
        this.message = message;
    }

    @Override
    public void onSchedule() {

    }

    @Override
    public void initialize() {

    }


    @Override
    public void update() {
    }

    @Override
    public void onFinish() {
        System.out.println(message);

    }
}
