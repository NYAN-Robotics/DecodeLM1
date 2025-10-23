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
import org.firstinspires.ftc.teamcode.utilities.robot.command.movement.MovementCommand;
import org.firstinspires.ftc.teamcode.utilities.robot.movement.MovementConstants;

/**
 * Test teleop path
 */
@Autonomous(name = "Awesome Red Auto")
public class TestAutoRed extends LinearOpMode {
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
                                new Pose(0, 10, Math.PI * 11 / 32),
                                new MovementConstants()
                        ),
                        new SequentialCommandGroup (
                                new YieldCommand(2000)
                        )
                ),
                new ParallelCommandGroup (
                        new MovementCommand (
                                new Pose(0, 10, Math.PI * 11 / 32),
                                new Pose(22, 26, 0),
                                new MovementConstants()
                        ),
                        new SequentialCommandGroup(
                                new YieldCommand(2000)
                        )
                ),
                new ParallelCommandGroup (
                        new MovementCommand (
                                new Pose(22, 26, 0),
                                new Pose(0, 10, Math.PI * 11 / 32),
                                new MovementConstants()
                        ),
                        new SequentialCommandGroup (
                                new YieldCommand(2000)
                        )
                ),
                new ParallelCommandGroup (
                        new MovementCommand (
                                new Pose(0, 10, Math.PI * 11 / 32),
                                new Pose(22, 47, 0),
                                new MovementConstants()
                        ),
                        new SequentialCommandGroup (
                                new YieldCommand(2000)
                        )
                ),
                new ParallelCommandGroup (
                        new MovementCommand (
                                new Pose(22, 47, 0),
                                new Pose(0, 10, Math.PI * 11 / 32),
                                new MovementConstants()
                        ),
                        new SequentialCommandGroup (
                                new YieldCommand(3000)
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