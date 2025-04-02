package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.utilities.math.linearalgebra.Pose;
import org.firstinspires.ftc.teamcode.utilities.robot.RobotEx;
import org.firstinspires.ftc.teamcode.utilities.robot.command.framework.commandtypes.OneTimeCommand;
import org.firstinspires.ftc.teamcode.utilities.robot.command.framework.commandtypes.ParallelCommandGroup;
import org.firstinspires.ftc.teamcode.utilities.robot.command.framework.commandtypes.SequentialCommandGroup;
import org.firstinspires.ftc.teamcode.utilities.robot.command.framework.commandtypes.YieldCommand;
import org.firstinspires.ftc.teamcode.utilities.robot.command.movement.MovementCommand;
import org.firstinspires.ftc.teamcode.utilities.robot.movement.MovementConstants;
import org.firstinspires.ftc.teamcode.utilities.robot.movement.PIDDrive;
import org.firstinspires.ftc.teamcode.utilities.robot.subsystems.Outtake;

/**
 * Example teleop code for a basic mecanum drive
 */
@Disabled
@Autonomous(name = "Drive Forward")
public class DriveForward extends LinearOpMode {

    // Create new Instance of the robot
    RobotEx robot = RobotEx.getInstance();

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.setMsTransmissionInterval(500);

        // Initialize the robot
        robot.init(this, telemetry);

        SequentialCommandGroup commands = new SequentialCommandGroup(
                new ParallelCommandGroup(
                        new MovementCommand(
                            new Pose(0, 0, Math.PI / 2),
                            new Pose(0, 40, Math.PI),
                            new MovementConstants()
                        ),
                        new SequentialCommandGroup(
                                new YieldCommand(500),
                            new OneTimeCommand(() -> robot.theOuttake.setSlidesState(Outtake.OuttakeSlidesStates.SAMPLES))
                        )
                ),
                new ParallelCommandGroup(
                        new MovementCommand(
                                new Pose(0, 40, Math.PI),
                                new Pose(0, 0, Math.PI / 2),
                                new MovementConstants()
                        ),
                        new SequentialCommandGroup(
                            new YieldCommand(500),
                            new OneTimeCommand(() -> robot.theOuttake.setSlidesState(Outtake.OuttakeSlidesStates.DEFAULT))
                        )

                )
                );

        waitForStart();

        // Notify subsystems before loop
        robot.postStart();

        if (isStopRequested()) return;

        // Initialize variables for loop
        Gamepad currentFrameGamepad1 = new Gamepad();
        Gamepad currentFrameGamepad2 = new Gamepad();

        Gamepad previousFrameGamepad1 = new Gamepad();
        Gamepad previousFrameGamepad2 = new Gamepad();

        PIDDrive drive = new PIDDrive(robot, this, telemetry);

        robot.theLocalizer.setPose(new Pose(0, 0, Math.PI / 2));

        robot.pause(0.5);
        /*
        MovementCommandCache initialCommand = new MovementCommandCache(
                new Pose(0, 0, Math.PI / 2),
                new Pose(0, 40, Math.PI),
                new MovementConstants()
        );

        MovementCommandCache returnCommand = new MovementCommandCache(
                new Pose(0, 40, Math.PI),
                new Pose(0, 0, Math.PI / 2),
                new MovementConstants()
        );

        drive.gotoPoint(initialCommand);
        drive.gotoPoint(returnCommand);

         */



        robot.theCommandScheduler.scheduleCommand(commands);

        while (!isStopRequested()) {
            robot.update();
        }

    }
}
