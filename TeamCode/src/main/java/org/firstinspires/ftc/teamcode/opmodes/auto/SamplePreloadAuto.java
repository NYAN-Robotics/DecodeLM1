package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.utilities.math.linearalgebra.Pose;
import org.firstinspires.ftc.teamcode.utilities.robot.RobotEx;
import org.firstinspires.ftc.teamcode.utilities.robot.movement.MovementConstants;
import org.firstinspires.ftc.teamcode.utilities.robot.movement.PIDDrive;

/**
 * Example teleop code for a basic mecanum drive
 */

@Autonomous(name = "SamplePreloadAuto")
public class SamplePreloadAuto extends LinearOpMode {

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
        robot.postInit();

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

        robot.odometry.setPose(new Pose(0, 0, -Math.PI / 2));



        robot.update();
        robot.pause(0.5);
        drive.gotoPoint(new Pose(0, 30, -Math.PI / 2));
        drive.gotoPoint(new Pose(-20, 10, 0));
        drive.gotoPoint(new Pose(-22, 23, Math.toRadians(90 + 65)));
        robot.pause(1);
        drive.gotoPoint(new Pose(-45, 10, Math.PI / 4));
        robot.pause(1);
        drive.gotoPoint(new Pose(-20, 10, Math.PI/2));
        drive.gotoPoint(new Pose(-32, 33, Math.PI));
        drive.gotoPoint(new Pose(-43, 9, Math.PI / 4));
        drive.gotoPoint(new Pose(-20, 10, Math.PI/2));
        drive.gotoPoint(new Pose(-42, 34, Math.PI));
        drive.gotoPoint(new Pose(-40, 8, Math.PI / 4));






        while (!isStopRequested()) {
            robot.update();
        }





    }
}
