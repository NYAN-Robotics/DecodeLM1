package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.utilities.math.linearalgebra.Pose;
import org.firstinspires.ftc.teamcode.utilities.robot.DriveConstants;
import org.firstinspires.ftc.teamcode.utilities.robot.RobotEx;
import org.firstinspires.ftc.teamcode.utilities.robot.movement.MovementCommand;
import org.firstinspires.ftc.teamcode.utilities.robot.movement.MovementConstants;
import org.firstinspires.ftc.teamcode.utilities.robot.movement.MovementStateCommand;
import org.firstinspires.ftc.teamcode.utilities.robot.movement.PIDDrive;

/**
 * Example teleop code for a basic mecanum drive
 */

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

        robot.odometry.setPose(new Pose(0, 0, Math.PI / 2));

        robot.pause(0.5);

        MovementCommand initialCommand = new MovementCommand(
                new Pose(0, 0, Math.PI / 2),
                new Pose(0, 40, Math.PI / 2),
                new MovementConstants(PIDDrive.vMax, PIDDrive.aMax, DriveConstants.MAX_CORRECTION_TIME, PIDDrive.kV, PIDDrive.kA)
        );

        MovementCommand returnCommand = new MovementCommand(
                new Pose(0, 40, Math.PI / 2),
                new Pose(0, 0, Math.PI / 2),
                new MovementConstants(PIDDrive.vMax, PIDDrive.aMax, DriveConstants.MAX_CORRECTION_TIME, PIDDrive.kV, PIDDrive.kA)
        );


        while (!isStopRequested()) {
            drive.gotoPoint(new Pose(0, 40, Math.PI));

            drive.gotoPoint(new Pose(0, 0, Math.PI / 4));
        }

        while (!isStopRequested()) {
            robot.update();
        }

    }
}
