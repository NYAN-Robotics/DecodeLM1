package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.utilities.math.linearalgebra.Pose;
import org.firstinspires.ftc.teamcode.utilities.robot.DriveConstants;
import org.firstinspires.ftc.teamcode.utilities.robot.RobotEx;
import org.firstinspires.ftc.teamcode.utilities.robot.command.framework.commandtypes.OneTimeCommand;
import org.firstinspires.ftc.teamcode.utilities.robot.command.framework.commandtypes.ParallelCommandGroup;
import org.firstinspires.ftc.teamcode.utilities.robot.command.framework.commandtypes.SequentialCommandGroup;
import org.firstinspires.ftc.teamcode.utilities.robot.command.framework.commandtypes.YieldCommand;
import org.firstinspires.ftc.teamcode.utilities.robot.command.movement.MovementCommand;
import org.firstinspires.ftc.teamcode.utilities.robot.movement.MovementConstants;
import org.firstinspires.ftc.teamcode.utilities.robot.movement.PIDDrive;
import org.firstinspires.ftc.teamcode.utilities.robot.subsystems.Intake;
import org.firstinspires.ftc.teamcode.utilities.robot.subsystems.Outtake;

/**
 * Example teleop code for a basic mecanum drive
 */
@Disabled
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

        robot.theIntake.setDisableOuttake(true);

        SequentialCommandGroup commands = new SequentialCommandGroup(
                new OneTimeCommand(() -> robot.theOuttake.setSlidesState(Outtake.OuttakeSlidesStates.SAMPLES)),
                new MovementCommand(
                        new Pose(-37.6, -61.8, Math.PI / 2),
                        new Pose(-54.5, -56, Math.PI/4),
                        new MovementConstants()
                ),
                new OneTimeCommand(() -> robot.theOuttake.setCurrentClawState(Outtake.OuttakeClawStates.DEFAULT)),
                new ParallelCommandGroup(
                    new MovementCommand(
                            new Pose(-54.5, -56, Math.PI / 4),
                            new Pose(-46.5, -50, Math.PI / 2),
                            new MovementConstants()
                    ),
                    new SequentialCommandGroup(
                            new YieldCommand(250),
                            new OneTimeCommand(() -> robot.theOuttake.reset())
                    )
                ),
                new OneTimeCommand(() -> robot.theIntake.setTargetLinkageState(Intake.LinkageStates.EXTENDED)),
                new OneTimeCommand(() -> robot.theIntake.setIntakeState(Intake.IntakeState.EXTENDED)),
                new OneTimeCommand(() -> robot.theIntake.setIntakeMotorState(Intake.IntakeMotorStates.INTAKING)),
                new YieldCommand(3000, robot.theIntake::containsSampleColorSensor),
                new YieldCommand(500),
                new OneTimeCommand(() -> robot.theIntake.returnSlides()),
                new ParallelCommandGroup(
                    new MovementCommand(
                            new Pose(-46.5, -50, Math.PI / 2),
                            new Pose(-54, -56, Math.PI / 4),
                            new MovementConstants()
                    ),
                    new SequentialCommandGroup(
                            new YieldCommand(250),
                            new YieldCommand(robot.theOuttake::atTargetPosition),
                            new YieldCommand(robot.theIntake::linkageAtHome),
                            new YieldCommand(150),
                            new OneTimeCommand(() -> robot.theOuttake.setCurrentClawState(Outtake.OuttakeClawStates.CLOSED)),
                            new YieldCommand(250),
                            new OneTimeCommand(() -> robot.theIntake.setTargetHolderState(Intake.SampleHolderState.DEFAULT)),
                            new YieldCommand(250),
                            new OneTimeCommand(() -> robot.theOuttake.setSlidesState(Outtake.OuttakeSlidesStates.SAMPLES))
                    )
                ),
                new YieldCommand(2000, () -> robot.theOuttake.getSlidesState() == Outtake.OuttakeSlidesStates.SAMPLES),
                new YieldCommand(() -> robot.theOuttake.atTargetPosition()),
                new YieldCommand(1000),
                new OneTimeCommand(() -> robot.theOuttake.setCurrentClawState(Outtake.OuttakeClawStates.DEFAULT)),
                new YieldCommand(100),
                new ParallelCommandGroup(
                        new MovementCommand(
                            new Pose(-54, -56, Math.PI / 4),
                            new Pose(-57, -47.8, Math.PI / 2),
                            new MovementConstants()
                        ),
                        new SequentialCommandGroup(
                                new YieldCommand(1000),
                                new OneTimeCommand(() -> robot.theIntake.setTargetLinkageState(Intake.LinkageStates.EXTENDED)),
                                new OneTimeCommand(() -> robot.theIntake.setIntakeState(Intake.IntakeState.EXTENDED)),
                                new OneTimeCommand(() -> robot.theIntake.setIntakeMotorState(Intake.IntakeMotorStates.INTAKING))
                        ),
                        new SequentialCommandGroup(
                                new YieldCommand(100),
                                new OneTimeCommand(() -> robot.theOuttake.reset())
                        )
                ),

                new YieldCommand(3000, robot.theIntake::containsSampleColorSensor),
                new YieldCommand(500),
                new OneTimeCommand(() -> robot.theIntake.returnSlides()),
                new ParallelCommandGroup(
                        new MovementCommand(
                                new Pose(-57, -47.8, Math.PI / 2),
                                new Pose(-54, -56, Math.PI / 4),
                                new MovementConstants()
                        ),
                        new SequentialCommandGroup(
                                new YieldCommand(250),
                                new YieldCommand(robot.theOuttake::atTargetPosition),
                                new YieldCommand(robot.theIntake::linkageAtHome),
                                new YieldCommand(100),
                                new OneTimeCommand(() -> robot.theOuttake.setCurrentClawState(Outtake.OuttakeClawStates.CLOSED)),
                                new YieldCommand(250),
                                new OneTimeCommand(() -> robot.theIntake.setTargetHolderState(Intake.SampleHolderState.DEFAULT)),
                                new YieldCommand(250),
                                new OneTimeCommand(() -> robot.theOuttake.setSlidesState(Outtake.OuttakeSlidesStates.SAMPLES))
                        )
                ),
                new YieldCommand(2000, () -> robot.theOuttake.getSlidesState() == Outtake.OuttakeSlidesStates.SAMPLES),
                new YieldCommand(() -> robot.theOuttake.atTargetPosition()),
                new YieldCommand(1000),
                new OneTimeCommand(() -> robot.theOuttake.setCurrentClawState(Outtake.OuttakeClawStates.DEFAULT)),
                new YieldCommand(100),

                new ParallelCommandGroup(
                        new MovementCommand(
                                new Pose(-54, -56, Math.PI / 4),
                                new Pose(-40, -28, Math.PI),
                                new MovementConstants()
                        ),
                        new SequentialCommandGroup(
                                new YieldCommand(100),
                                new OneTimeCommand(() -> robot.theOuttake.reset())
                        )
                ),
                new SequentialCommandGroup(
                        new OneTimeCommand(() -> robot.theIntake.setTargetLinkageState(Intake.LinkageStates.EXTENDED)),
                        new OneTimeCommand(() -> robot.theIntake.setIntakeState(Intake.IntakeState.EXTENDED)),
                        new OneTimeCommand(() -> robot.theIntake.setIntakeMotorState(Intake.IntakeMotorStates.INTAKING))
                ),
                new YieldCommand(3000, robot.theIntake::containsSampleColorSensor),
                new YieldCommand(500),
                new OneTimeCommand(() -> robot.theIntake.returnSlides()),
                new ParallelCommandGroup(
                        new MovementCommand(
                                new Pose(-40, -28, Math.PI),
                                new Pose(-54, -56, Math.PI / 4),
                                new MovementConstants()
                        ),
                        new SequentialCommandGroup(
                                new YieldCommand(250),
                                new YieldCommand(robot.theOuttake::atTargetPosition),
                                new YieldCommand(robot.theIntake::linkageAtHome),
                                new YieldCommand(100),
                                new OneTimeCommand(() -> robot.theOuttake.setCurrentClawState(Outtake.OuttakeClawStates.CLOSED)),
                                new YieldCommand(250),
                                new OneTimeCommand(() -> robot.theIntake.setTargetHolderState(Intake.SampleHolderState.DEFAULT)),
                                new YieldCommand(250),
                                new OneTimeCommand(() -> robot.theOuttake.setSlidesState(Outtake.OuttakeSlidesStates.SAMPLES))
                        )
                ),
                new YieldCommand(2000, () -> robot.theOuttake.getSlidesState() == Outtake.OuttakeSlidesStates.SAMPLES),
                new YieldCommand(() -> robot.theOuttake.atTargetPosition()),
                new YieldCommand(1000),
                new OneTimeCommand(() -> robot.theOuttake.setCurrentClawState(Outtake.OuttakeClawStates.DEFAULT)),
                new YieldCommand(100),

                new ParallelCommandGroup(
                        new MovementCommand(
                                new Pose(-55, -56.5, Math.PI / 4),
                                new Pose(-40, -11, Math.PI),
                                new MovementConstants(60, 60, 0, DriveConstants.K_V, DriveConstants.K_A)
                        ),
                        new SequentialCommandGroup(
                                new YieldCommand(100),
                                new OneTimeCommand(() -> robot.theOuttake.reset())
                        )
                ),

                new MovementCommand(
                        new Pose(-40, -11, Math.PI),
                        new Pose(-15, -11, Math.PI),
                        new MovementConstants(40, 15, 0, DriveConstants.K_V, DriveConstants.K_A)
                ),

                new OneTimeCommand(() -> robot.theIntake.setTargetLinkageState(Intake.LinkageStates.EXTENDED)),
                new YieldCommand(250),
                new OneTimeCommand(() -> robot.theOuttake.setCurrentOuttakeState(Outtake.OuttakeServoState.AUTO_PARK))
        );



        robot.theOuttake.setCurrentClawState(Outtake.OuttakeClawStates.CLOSED);

        robot.theIntake.leftServo.setPosition(Intake.LinkageStates.DEFAULT.position);
        robot.theIntake.rightServo.setPosition(Intake.LinkageStates.DEFAULT.position);

        while (opModeInInit()) {
            if (gamepad1.cross) {
                robot.theOuttake.clawServo.setPosition(Outtake.OuttakeClawStates.CLOSED.position);
            }
        }
        waitForStart();

        robot.theOuttake.setCurrentClawState(Outtake.OuttakeClawStates.CLOSED);
        robot.theOuttake.setCurrentOuttakeState(Outtake.OuttakeServoState.AUTO_DEFAULT);

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


        ElapsedTime timer = new ElapsedTime();

        PIDDrive drive = new PIDDrive(robot, this, telemetry);

        robot.theLocalizer.setPose(new Pose(-37.6, -61.8, Math.PI / 2));

        robot.pause(0.05);



        drive.gotoPoint(new Pose(-54, -55, Math.PI / 4));
        /*
        robot.pause(2);
        drive.gotoPoint(new Pose(-48.5, -52.2, Math.PI / 2));
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
        drive.gotoPoint(new Pose(-40, -5, -Math.PI));
         */

        // robot.pause(2);


        // drive.gotoPoint(new Pose(-20, -5, -Math.PI), new MovementConstants(10, 10, 3));

        robot.theCommandScheduler.scheduleCommand(commands);






        while (!isStopRequested()) {
            robot.update();
        }















        while (!isStopRequested()) {
            robot.update();
        }





    }
}
