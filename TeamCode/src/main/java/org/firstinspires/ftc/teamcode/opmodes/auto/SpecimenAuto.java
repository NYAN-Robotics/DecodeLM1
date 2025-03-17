package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.utilities.math.linearalgebra.Pose;
import org.firstinspires.ftc.teamcode.utilities.robot.RobotEx;
import org.firstinspires.ftc.teamcode.utilities.robot.command.framework.commandtypes.SequentialCommandGroup;
import org.firstinspires.ftc.teamcode.utilities.robot.command.framework.commandtypes.YieldCommand;
import org.firstinspires.ftc.teamcode.utilities.robot.command.movement.MovementCommand;
import org.firstinspires.ftc.teamcode.utilities.robot.movement.MovementConstants;
import org.firstinspires.ftc.teamcode.utilities.robot.movement.PIDDrive;
import org.firstinspires.ftc.teamcode.utilities.robot.subsystems.Intake;

/**
 * Example teleop code for a basic mecanum drive
 */

@Autonomous(name = "Specimen Auto")
public class SpecimenAuto extends LinearOpMode {

    // Create new Instance of the robot
    RobotEx robot = RobotEx.getInstance();

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.setMsTransmissionInterval(500);

        // Initialize the robot
        robot.init(this, telemetry);


        waitForStart();

        robot.theIntake.setIntakeState(Intake.IntakeState.EXTENDED);
        // Notify subsystems before loop
        robot.postStart();

        if (isStopRequested()) return;

        // Initialize variables for loop
        Gamepad currentFrameGamepad1 = new Gamepad();
        Gamepad currentFrameGamepad2 = new Gamepad();

        Gamepad previousFrameGamepad1 = new Gamepad();
        Gamepad previousFrameGamepad2 = new Gamepad();

        // robot.drivetrain.enableAntiTip();

        robot.update();

        ElapsedTime e = new ElapsedTime();

        // robot.localizer.setPose(new Pose(-59, 15, Math.PI/2), true);


        PIDDrive drive = new PIDDrive(robot, this, telemetry);

        robot.theLocalizer.setPose(new Pose(9.3, -62.2, Math.PI / 2));

        // Points to go to
        // (1.77, -32.1, Math.PI / 2)
        // (31.5, -40.29, 0.6478)
        // (33.58, -48.74, -0.6355)
        // (33.26, -45.7764, 0.6466)
        // (37.24, -49.1045, -0.87)
        // (45.4, -42.9, 0.6302)
        // (45.1, -49.8, -0.8727)
        // (34.8309, -62.36, 1.6119)
        // (2.6, -33.82, 1.5975)


        SequentialCommandGroup commands = new SequentialCommandGroup(
                new MovementCommand(new Pose(9.3, -62.2, Math.PI / 2), new Pose(1.77, -32.1, Math.PI / 2), new MovementConstants()),
                new YieldCommand(500),
                new MovementCommand(new Pose(1.77, -32.1, Math.PI / 2), new Pose(31.5, -40.29, 0.6478), new MovementConstants()),
                new YieldCommand(500),
                new MovementCommand(new Pose(31.5, -40.29, 0.6478), new Pose(33.58, -48.74, -0.6355), new MovementConstants()),
                new YieldCommand(500),
                new MovementCommand(new Pose(33.58, -48.74, -0.6355), new Pose(33.26, -45.7764, 0.6466), new MovementConstants()),
                new YieldCommand(500),
                new MovementCommand(new Pose(33.26, -45.7764, 0.6466), new Pose(37.24, -49.1045, -0.87), new MovementConstants()),
                new YieldCommand(500),
                new MovementCommand(new Pose(37.24, -49.1045, -0.87), new Pose(45.4, -42.9, 0.6302), new MovementConstants()),
                new YieldCommand(500),
                new MovementCommand(new Pose(45.4, -42.9, 0.6302), new Pose(45.1, -49.8, -0.8727), new MovementConstants()),
                new YieldCommand(500),
                new MovementCommand(new Pose(45.1, -49.8, -0.8727), new Pose(34.8309, -62.36, 1.6119), new MovementConstants()),
                new YieldCommand(500),
                new MovementCommand(new Pose(34.8309, -62.36, 1.6119), new Pose(2.6, -33.82, 1.5975), new MovementConstants())
        );

        robot.theCommandScheduler.scheduleCommand(commands);










        while (!isStopRequested()) {
            robot.update();
        }

    }
}
