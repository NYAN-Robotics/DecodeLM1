package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
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

@TeleOp(name = "Main Teleop")
public class MainTeleop extends LinearOpMode {

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

        double frameTime = 0;

        while (!robot.stopRequested) {

            e.reset();
            // Retain information about the previous frame's gamepad
            previousFrameGamepad1.copy(currentFrameGamepad1);
            previousFrameGamepad2.copy(currentFrameGamepad2);

            currentFrameGamepad1.copy(gamepad1);
            currentFrameGamepad2.copy(gamepad2);


            robot.drivetrain.robotCentricDriveFromGamepad(
                    -currentFrameGamepad1.left_stick_y,
                    currentFrameGamepad1.left_stick_x,
                    currentFrameGamepad1.right_stick_x
            );

            if (currentFrameGamepad2.right_trigger > 0) {
                robot.outtake.setLiftPower(currentFrameGamepad2.right_trigger);
            } else {
                robot.outtake.setLiftPower(-currentFrameGamepad2.left_trigger);
            }

            if (currentFrameGamepad1.right_trigger > 0.05) {
                robot.intake.incrementPositionByVelocity(currentFrameGamepad1.right_trigger, frameTime/1000);
            } else if (currentFrameGamepad1.left_trigger > 0.05) {
                robot.intake.incrementPositionByVelocity(-currentFrameGamepad1.left_trigger, frameTime/1000);
            }

            if (currentFrameGamepad1.left_bumper && !previousFrameGamepad1.left_bumper) {
                robot.intake.setTargetLinkageState(Intake.LinkageStates.DEFAULT);
                robot.intake.setIntakeState(Intake.IntakeState.DEFAULT);
            }

            if (currentFrameGamepad1.right_bumper && !previousFrameGamepad1.right_bumper) {

                if (robot.intake.currentLinkageState == Intake.LinkageStates.DEFAULT) {
                    robot.intake.setTargetLinkageState(Intake.LinkageStates.EXTENDED);
                    robot.intake.setIntakeState(Intake.IntakeState.DEFAULT);
                } else {
                    robot.intake.setIntakeState(Intake.IntakeState.EXTENDED);
                }
            }

            if (currentFrameGamepad1.circle) {
                robot.intake.setIntakeState(Intake.IntakeState.DEFAULT);
                robot.intake.reverseIntake();
            } else if (previousFrameGamepad1.circle && !currentFrameGamepad1.circle) {
                robot.intake.setIntakeState(Intake.IntakeState.EXTENDED);
            }

            if (currentFrameGamepad2.dpad_up && !previousFrameGamepad2.dpad_up) {
                robot.outtake.setSlidesState(Outtake.OuttakeSlidesStates.SAMPLES);
                robot.outtake.setCurrentClawState(Outtake.OuttakeClawStates.CLOSED);
            }

            if (currentFrameGamepad2.dpad_left && !previousFrameGamepad2.dpad_left) {
                robot.outtake.setSlidesState(Outtake.OuttakeSlidesStates.SPECIMENS);
                robot.outtake.setCurrentClawState(Outtake.OuttakeClawStates.CLOSED);
            }

            if (currentFrameGamepad2.dpad_right && !previousFrameGamepad2.dpad_right) {
                robot.outtake.setSlidesState(Outtake.OuttakeSlidesStates.SPECIMENS_DROP);
            }

            if (currentFrameGamepad2.dpad_down && !previousFrameGamepad2.dpad_down) {
                robot.outtake.setSlidesState(Outtake.OuttakeSlidesStates.HOVER);
                robot.outtake.setCurrentRotationState(Outtake.OuttakeRotationStates.ROTATED);
                robot.outtake.setCurrentClawState(Outtake.OuttakeClawStates.FULL_DEFAULT);
            }

            if (currentFrameGamepad2.right_bumper && !previousFrameGamepad2.right_bumper) {
                robot.outtake.setCurrentClawState(Outtake.OuttakeClawStates.CLOSED);
            }

            if (currentFrameGamepad2.left_bumper && !previousFrameGamepad2.left_bumper) {
                robot.outtake.setCurrentClawState(Outtake.OuttakeClawStates.DEFAULT);
            }

            if (currentFrameGamepad2.triangle && !previousFrameGamepad2.triangle) {
                if (robot.outtake.getSlidesState() != Outtake.OuttakeSlidesStates.DEFAULT) {
                    robot.outtake.setCurrentOuttakeState(Outtake.OuttakeServoState.BACK_PICKUP);
                }
            }

            if (currentFrameGamepad2.circle && !previousFrameGamepad2.circle) {
                robot.outtake.setCurrentOuttakeState(Outtake.OuttakeServoState.EXTENDED);
            }

            if (currentFrameGamepad2.square && !previousFrameGamepad2.square) {
                robot.outtake.setSlidesState(Outtake.OuttakeSlidesStates.DEFAULT);
                robot.outtake.setCurrentOuttakeState(Outtake.OuttakeServoState.DEFAULT);
                robot.outtake.setCurrentClawState(Outtake.OuttakeClawStates.DEFAULT);
                robot.outtake.setCurrentRotationState(Outtake.OuttakeRotationStates.DEFAULT);
            }

            if (currentFrameGamepad1.square && !previousFrameGamepad1.square) {
                robot.intake.setIntakeState(Intake.IntakeState.EXTENDED);
            }

            if (currentFrameGamepad1.cross && !previousFrameGamepad1.cross) {
                robot.intake.setIntakeState(Intake.IntakeState.PUSH_DOWN);
            }

            if (currentFrameGamepad1.triangle && !previousFrameGamepad1.triangle) {
                robot.intake.setIntakeState(Intake.IntakeState.UP);
            }



            /*

            if (currentFrameGamepad2.triangle && !previousFrameGamepad2.triangle) {
                robot.outtake.setCurrentOuttakeState(Outtake.OuttakeState.DEFAULT);
            }

            if (currentFrameGamepad2.circle && !previousFrameGamepad2.circle) {
                robot.outtake.setCurrentOuttakeState(Outtake.OuttakeState.EXTENDED);
            }

            if (currentFrameGamepad2.cross && !previousFrameGamepad2.cross) {
                robot.outtake.setCurrentRotationState(Outtake.OuttakeRotationStates.DEFAULT);
            }

            if (currentFrameGamepad2.square && !previousFrameGamepad2.square) {
                robot.outtake.setCurrentRotationState(Outtake.OuttakeRotationStates.ROTATED);
            }

            if (currentFrameGamepad2.right_bumper && !previousFrameGamepad2.right_bumper) {
                robot.outtake.setCurrentClawState(Outtake.OuttakeClawStates.CLOSED);
            }

            if (currentFrameGamepad2.left_bumper && !previousFrameGamepad2.left_bumper) {
                robot.outtake.setCurrentClawState(Outtake.OuttakeClawStates.DEFAULT);
            }
            */

            frameTime = robot.update();

            Pose currentPose = robot.odometry.getPose();

            telemetry.addData("X Position: ", currentPose.getX());
            telemetry.addData("Y Position: ", currentPose.getY());
            telemetry.addData("Heading: ", currentPose.getHeading());

            telemetry.addData("Frame Time: ", MathHelper.truncate(frameTime, 3));
        }
    }
}
