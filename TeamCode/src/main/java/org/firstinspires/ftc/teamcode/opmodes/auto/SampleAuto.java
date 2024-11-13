package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.utilities.math.MathHelper;
import org.firstinspires.ftc.teamcode.utilities.math.linearalgebra.Pose;
import org.firstinspires.ftc.teamcode.utilities.robot.RobotEx;
import org.firstinspires.ftc.teamcode.utilities.robot.movement.MovementConstants;
import org.firstinspires.ftc.teamcode.utilities.robot.movement.PIDDrive;
import org.firstinspires.ftc.teamcode.utilities.robot.subsystems.Intake;
import org.firstinspires.ftc.teamcode.utilities.robot.subsystems.Outtake;

/**
 * Example teleop code for a basic mecanum drive
 */

@Autonomous(name = "Sample Preload Auto")
public class SampleAuto extends LinearOpMode {

    // Create new Instance of the robot
    RobotEx robot = RobotEx.getInstance();

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.setMsTransmissionInterval(500);

        // Initialize the robot
        robot.init(this, telemetry);


        waitForStart();

        robot.outtake.setCurrentClawState(Outtake.OuttakeClawStates.CLOSED);

        robot.outtake.setCurrentOuttakeState(Outtake.OuttakeServoState.AUTO_DEFAULT);
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


        ElapsedTime timer = new ElapsedTime();

        PIDDrive drive = new PIDDrive(robot, this, telemetry);

        robot.odometry.setPose(new Pose(-37.6, -61.8, Math.PI / 2));

        robot.pause(0.05);

        drive.gotoPoint(new Pose(-54, -55, Math.PI / 4));
        robot.pause(2);
        drive.gotoPoint(new Pose(-48.2, -52.2, Math.PI / 2));
        robot.pause(2);
        drive.gotoPoint(new Pose(-54, -55, Math.PI / 4));
        robot.pause(2);
        drive.gotoPoint(new Pose(-57.6, -47.8, Math.PI / 2));
        robot.pause(2);
        drive.gotoPoint(new Pose(-54, -55, Math.PI / 4));
        robot.pause(2);
        drive.gotoPoint(new Pose(-44, -27, Math.PI));
        robot.pause(2);
        drive.gotoPoint(new Pose(-54, -55, Math.PI / 4));
        robot.pause(2);
        drive.gotoPoint(new Pose(-40, -10, 0));
        drive.gotoPoint(new Pose(-30, -10, Math.PI / 4), new MovementConstants(10, 10, 0));






        while (!isStopRequested()) {
            robot.update();
        }















        while (!isStopRequested()) {
            robot.update();
        }





    }
}
