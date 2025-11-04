package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.utilities.math.linearalgebra.Pose;
import org.firstinspires.ftc.teamcode.utilities.robot.RobotEx;
import org.firstinspires.ftc.teamcode.utilities.robot.command.framework.commandtypes.ParallelCommandGroup;
import org.firstinspires.ftc.teamcode.utilities.robot.command.framework.commandtypes.SequentialCommandGroup;
import org.firstinspires.ftc.teamcode.utilities.robot.command.framework.commandtypes.YieldCommand;
import org.firstinspires.ftc.teamcode.utilities.robot.movement.MovementCommand;
import org.firstinspires.ftc.teamcode.utilities.robot.movement.MovementConstants;

/**
 * Test teleop path
 */
@Autonomous(name = "Low Red Auto")
public class LowRedAuto extends LinearOpMode {
    RobotEx robot = RobotEx.getInstance();

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.setMsTransmissionInterval(500);

        robot.init(this, telemetry);

        SequentialCommandGroup commands = new SequentialCommandGroup(
                new ParallelCommandGroup (
                        new MovementCommand (
                                new Pose(0, 0, Math.PI / 2),
                                new Pose(0, 50, 3 * Math.PI / 2),
                                new MovementConstants()
                        ),
                        new SequentialCommandGroup (
                                new YieldCommand(2000)
                        )
                ),
                new ParallelCommandGroup (
                        new MovementCommand (
                                new Pose(0, 50, 3 * Math.PI / 2),
                                new Pose(-23, 38,  5 * Math.PI / 4),
                                new MovementConstants()
                        ),
                        new SequentialCommandGroup(
                                new YieldCommand(2000)
                        )
                ),
                new ParallelCommandGroup (
                        new MovementCommand (
                                new Pose(-23, 38,  5 * Math.PI / 4),
                                new Pose(0, 50, 3 * Math.PI / 2),
                                new MovementConstants()
                        ),
                        new SequentialCommandGroup (
                                new YieldCommand(2000)
                        )
                )
        );

        waitForStart();

        robot.postStart();

        if (isStopRequested()) return;

        robot.theLocalizer.setPose(new Pose(0, 0, Math.PI / 2));

        robot.pause(0.5);

        robot.theCommandScheduler.scheduleCommand(commands);

        while (!isStopRequested()) {
            robot.update();
        }
    }
}