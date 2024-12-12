package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.fasterxml.jackson.databind.ext.SqlBlobSerializer;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.utilities.math.MathHelper;
import org.firstinspires.ftc.teamcode.utilities.robot.RobotEx;
import org.firstinspires.ftc.teamcode.utilities.robot.command.framework.commandtypes.OneTimeCommand;
import org.firstinspires.ftc.teamcode.utilities.robot.command.framework.commandtypes.SequentialCommandGroup;
import org.firstinspires.ftc.teamcode.utilities.robot.command.framework.commandtypes.YieldCommand;
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
        robot.theIntake.setDisableOuttake(false);


        waitForStart();

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

        double frameTime = 0;

        while (!robot.stopRequested) {

            e.reset();
            // Retain information about the previous frame's gamepad
            previousFrameGamepad1.copy(currentFrameGamepad1);
            previousFrameGamepad2.copy(currentFrameGamepad2);

            currentFrameGamepad1.copy(gamepad1);
            currentFrameGamepad2.copy(gamepad2);


            robot.theDrivetrain.robotCentricDriveFromGamepad(
                    -currentFrameGamepad1.left_stick_y,
                    currentFrameGamepad1.left_stick_x,
                    currentFrameGamepad1.right_stick_x
            );

            if (currentFrameGamepad2.right_trigger > 0) {
                robot.theOuttake.setLiftPower(currentFrameGamepad2.right_trigger);
            } else {
                robot.theOuttake.setLiftPower(-currentFrameGamepad2.left_trigger);
            }

            if (currentFrameGamepad1.right_trigger > 0.05) {
                robot.theIntake.incrementPositionByVelocity(currentFrameGamepad1.right_trigger, frameTime/1000);
            } else if (currentFrameGamepad1.left_trigger > 0.05) {
                robot.theIntake.incrementPositionByVelocity(-currentFrameGamepad1.left_trigger, frameTime/1000);
            }

            if (currentFrameGamepad1.left_bumper && !previousFrameGamepad1.left_bumper) {
                robot.theIntake.returnSlides();
            }

            if (currentFrameGamepad1.right_bumper && !previousFrameGamepad1.right_bumper) {

                if (robot.theIntake.currentLinkageState == Intake.LinkageStates.DEFAULT) {
                    robot.theIntake.setTargetLinkageState(Intake.LinkageStates.EXTENDED);
                    robot.theIntake.setIntakeState(Intake.IntakeState.DEFAULT);
                } else {
                    robot.theIntake.setIntakeState(Intake.IntakeState.EXTENDED);
                    robot.theIntake.setIntakeMotorState(Intake.IntakeMotorStates.INTAKING);
                }
            }

            if (currentFrameGamepad1.circle && !previousFrameGamepad1.circle) {
                robot.theIntake.reverseIntake();
            }

            if (currentFrameGamepad2.dpad_up && !previousFrameGamepad2.dpad_up) {
                robot.theOuttake.setSlidesState(Outtake.OuttakeSlidesStates.SAMPLES);
                robot.theOuttake.setCurrentClawState(Outtake.OuttakeClawStates.CLOSED);
            }

            if (currentFrameGamepad2.dpad_left && !previousFrameGamepad2.dpad_left) {
                robot.theOuttake.setSlidesState(Outtake.OuttakeSlidesStates.SPECIMENS);
            }

            if (currentFrameGamepad2.dpad_right && !previousFrameGamepad2.dpad_right) {

                if (robot.theOuttake.getSlidesState() != Outtake.OuttakeSlidesStates.HANG) {
                    robot.theOuttake.setSlidesState(Outtake.OuttakeSlidesStates.HANG);
                } else {
                    robot.theCommandScheduler.scheduleCommand(
                            new SequentialCommandGroup(
                                    new OneTimeCommand(() -> robot.theOuttake.setCurrentOuttakeState(Outtake.OuttakeServoState.HANG_FINAL)),
                                    new YieldCommand(1000),
                                    new OneTimeCommand(() -> robot.theOuttake.setSlidesState(Outtake.OuttakeSlidesStates.HANG_FINAL))
                            )
                    );
                }
            }

            if (currentFrameGamepad2.dpad_down && !previousFrameGamepad2.dpad_down) {
                robot.theCommandScheduler.scheduleCommand(
                        new SequentialCommandGroup(
                                new OneTimeCommand(() -> robot.theOuttake.setSlidesState(Outtake.OuttakeSlidesStates.SPECIMEN_INITIAL_PICKUP)),
                                new YieldCommand(3000, robot.theOuttake::atTargetPosition),
                                new OneTimeCommand(() -> robot.theOuttake.setCurrentOuttakeState(Outtake.OuttakeServoState.SPECIMEN_PICKUP)),
                                new OneTimeCommand(() -> robot.theOuttake.setCurrentPivotState(Outtake.OuttakePivotStates.SPECIMEN_PICKUP)),
                                new OneTimeCommand(() -> robot.theOuttake.setCurrentRotationState(Outtake.OuttakeRotationStates.ROTATED)),
                                new YieldCommand(1000),
                                new OneTimeCommand(() -> robot.theOuttake.setSlidesState(Outtake.OuttakeSlidesStates.SPECIMEN_PICKUP))

                        )
                );
            }

            if (currentFrameGamepad2.right_bumper && !previousFrameGamepad2.right_bumper) {
                robot.theOuttake.setCurrentClawState(Outtake.OuttakeClawStates.CLOSED);
            }

            if (currentFrameGamepad2.left_bumper && !previousFrameGamepad2.left_bumper) {
                if (robot.theOuttake.getOuttakeServoState() == Outtake.OuttakeServoState.SPECIMEN_INITIAL) {
                    robot.theCommandScheduler.scheduleCommand(
                            new SequentialCommandGroup(
                                    new OneTimeCommand(() -> robot.theOuttake.setCurrentOuttakeState(Outtake.OuttakeServoState.SPECIMEN_DROP_FINAL)),
                                    new YieldCommand(500),
                                    new OneTimeCommand(() -> robot.theOuttake.setCurrentClawState(Outtake.OuttakeClawStates.DEFAULT)),
                                    new YieldCommand(500),
                                    new OneTimeCommand(() -> robot.theOuttake.setCurrentOuttakeState(Outtake.OuttakeServoState.SPECIMEN_INITIAL))
                            )
                    );
                } else {
                    robot.theOuttake.setCurrentClawState(Outtake.OuttakeClawStates.DEFAULT);
                }
            }

            if (currentFrameGamepad2.triangle && !previousFrameGamepad2.triangle) {
                robot.theOuttake.setCurrentPivotState(Outtake.OuttakePivotStates.DOWN);
            }

            /*
            if (currentFrameGamepad2.circle && !previousFrameGamepad2.circle) {
                robot.outtake.setCurrentOuttakeState(Outtake.OuttakeServoState.BACK_PICKUP);
            }*/

            if (currentFrameGamepad2.cross && !previousFrameGamepad2.cross) {
                robot.theOuttake.setCurrentOuttakeState(Outtake.OuttakeServoState.DEFAULT);
            }

            if (currentFrameGamepad2.square && !previousFrameGamepad2.square) {
                robot.theOuttake.setSlidesState(Outtake.OuttakeSlidesStates.DEFAULT);
                robot.theOuttake.setCurrentOuttakeState(Outtake.OuttakeServoState.DEFAULT);
                robot.theOuttake.setCurrentClawState(Outtake.OuttakeClawStates.DEFAULT);
                robot.theOuttake.setCurrentRotationState(Outtake.OuttakeRotationStates.DEFAULT);
                robot.theOuttake.setCurrentPivotState(Outtake.OuttakePivotStates.DEFAULT);
            }

            if (currentFrameGamepad1.cross && !previousFrameGamepad1.cross) {
                robot.theCommandScheduler.scheduleCommand(
                        new SequentialCommandGroup(
                                new OneTimeCommand(() -> robot.theIntake.setCurrentCowcatcherState(Intake.CowcatcherStates.ACTIVATED)),
                                new YieldCommand(400),
                                new OneTimeCommand(() -> robot.theIntake.setCurrentCowcatcherState(Intake.CowcatcherStates.DEFAULT)))
                        )


                        ;
            }

            frameTime = robot.update();

            telemetry.addData("Frame Time: ", MathHelper.truncate(frameTime, 3));
        }
    }
}
