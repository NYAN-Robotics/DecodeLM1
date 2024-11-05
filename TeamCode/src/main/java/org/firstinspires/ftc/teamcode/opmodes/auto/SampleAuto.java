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

        robot.odometry.setPose(new Pose(-7, 0, Math.PI / 2));

        robot.update();
        robot.pause(0.5);
        robot.outtake.setSlidesState(Outtake.OuttakeSlidesStates.SAMPLES);
        drive.gotoPoint(new Pose(-39.5, 8, Math.PI/4));
        robot.pause(0.25);
        robot.outtake.setCurrentClawState(Outtake.OuttakeClawStates.DEFAULT);
        robot.pause(0.25);
        drive.gotoPoint(new Pose(-43.5-1*Math.cos(1.23), 11.5-1*Math.sin(1.23), 1.23));
        robot.outtake.setCurrentOuttakeState(Outtake.OuttakeServoState.DEFAULT);
        robot.outtake.reset();
        robot.intake.setTargetLinkageState(Intake.LinkageStates.EXTENDED);
        robot.intake.setIntakeState(Intake.IntakeState.DEFAULT);
        robot.pause(1);
        robot.intake.setIntakeState(Intake.IntakeState.EXTENDED);
        robot.pause(0.5);
        drive.gotoPoint(new Pose(-43.5+6*Math.cos(1.23), 11.5+6*Math.sin(1.23), 1.23));
        robot.pause(1);
        robot.intake.setTargetLinkageState(Intake.LinkageStates.DEFAULT);
        drive.gotoPoint(new Pose(-39.5, 8, Math.PI/4));
        robot.pause(0.5);
        robot.outtake.setCurrentClawState(Outtake.OuttakeClawStates.CLOSED);
        robot.pause(0.5);
        robot.outtake.setSlidesState(Outtake.OuttakeSlidesStates.SAMPLES);
        robot.pause(1);

        timer.reset();

        while (timer.seconds() < 1) {
            robot.intake.reverseIntake();
            robot.update();
        }

        robot.outtake.setCurrentClawState(Outtake.OuttakeClawStates.DEFAULT);
        robot.pause(0.5);
        robot.outtake.reset();

        drive.gotoPoint(new Pose(-35-6*Math.cos(2.1), 19-6*Math.sin(2.1), 2.1));
        robot.outtake.setCurrentOuttakeState(Outtake.OuttakeServoState.DEFAULT);
        robot.outtake.reset();
        robot.intake.setTargetLinkageState(Intake.LinkageStates.EXTENDED);
        robot.intake.setIntakeState(Intake.IntakeState.DEFAULT);
        robot.pause(1);
        robot.intake.setIntakeState(Intake.IntakeState.EXTENDED);
        robot.pause(0.5);
        drive.gotoPoint(new Pose(-35+6*Math.cos(2.1), 19+6*Math.sin(2.1), 2.1));
        robot.pause(2);
        robot.intake.setTargetLinkageState(Intake.LinkageStates.DEFAULT);
        drive.gotoPoint(new Pose(-39.5, 8, Math.PI/4));
        robot.pause(0.5);
        robot.outtake.setCurrentClawState(Outtake.OuttakeClawStates.CLOSED);
        robot.pause(0.5);
        robot.outtake.setSlidesState(Outtake.OuttakeSlidesStates.SAMPLES);
        robot.pause(1);
        timer.reset();

        while (timer.seconds() < 1) {
            robot.intake.reverseIntake();
            robot.update();
        }
        robot.outtake.setCurrentClawState(Outtake.OuttakeClawStates.DEFAULT);
        robot.pause(0.5);
        robot.outtake.reset();











        while (!isStopRequested()) {
            robot.update();
        }





    }
}
