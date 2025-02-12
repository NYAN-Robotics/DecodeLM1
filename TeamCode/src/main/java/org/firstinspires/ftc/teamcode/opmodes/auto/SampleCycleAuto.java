package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.utilities.math.linearalgebra.Pose;
import org.firstinspires.ftc.teamcode.utilities.robot.Alliance;
import org.firstinspires.ftc.teamcode.utilities.robot.DriveConstants;
import org.firstinspires.ftc.teamcode.utilities.robot.Globals;
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

@Autonomous(name = "Sample Cycle Auto")
public class SampleCycleAuto extends LinearOpMode {

    // Create new Instance of the robot
    RobotEx robot = RobotEx.getInstance();

    public static Pose startPose = new Pose(-37.6, -61.8, Math.PI / 2);
    public static Pose initialDrop = new Pose(-54.5, -56, Math.PI / 4);
    public static Pose sample1 = new Pose(-53.5, -53.9, 1.4);
    public static Pose sample1Final = new Pose(-53.5 + Math.cos(1.4)*3, -53.9 + Math.sin(1.4)*3, 1.4);
    public static Pose sample1Drop = new Pose(-54, -56, Math.PI / 4);
    public static Pose sample2 = new Pose(-58, -47.8, Math.PI / 2);
    public static Pose sample2Drop = new Pose(-54, -56, Math.PI / 4);
    public static Pose sample3 = new Pose(-48, -46.5, 2.4);
    public static Pose sample3Final = new Pose(-48 + Math.cos(2.4)*3, -46.5 + Math.sin(2.4)*3, 2.4);
    public static Pose sample3Drop = new Pose(-54, -56, Math.PI / 4);
    public static Pose cycleInitial = new Pose(-40, -11, 0);
    public static Pose cycleSubmersible = new Pose(-23, -11, 0);
    public static Pose cycleDrop = new Pose(-55, -56.5, Math.PI / 4);
    public static Pose cycleStrafe = new Pose(-23, -14, 0);
    public static Pose cycleFinal = new Pose(-45, -45, Math.PI / 4);
    public static Pose parkInitial = new Pose(-40, -11, Math.PI);
    public static Pose parkFinal = new Pose(-17, -11, Math.PI);

    public static MovementConstants defaultMovementConstants = new MovementConstants();

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.setMsTransmissionInterval(500);

        // Initialize the robot
        robot.init(this, telemetry);

        robot.theIntake.setDisableOuttake(true);

        boolean doneWithInitial = false;


        SequentialCommandGroup commands = new SequentialCommandGroup(
                new OneTimeCommand(() -> robot.theOuttake.setSlidesState(Outtake.OuttakeSlidesStates.SAMPLES)),
                new MovementCommand(
                        startPose,
                        initialDrop,
                        defaultMovementConstants
                ),
                new OneTimeCommand(() -> robot.theOuttake.setCurrentClawState(Outtake.OuttakeClawStates.DEFAULT)),
                new ParallelCommandGroup(
                        new MovementCommand(
                                initialDrop,
                                sample1,
                                defaultMovementConstants
                        ),
                        new SequentialCommandGroup(
                                new YieldCommand(250),
                                new OneTimeCommand(() -> robot.theIntake.setTargetLinkageState(Intake.LinkageStates.EXTENDED))
                        )
                ),
                new OneTimeCommand(() -> robot.theIntake.setIntakeState(Intake.IntakeState.EXTENDED)),
                new OneTimeCommand(() -> robot.theIntake.setIntakeMotorState(Intake.IntakeMotorStates.INTAKING)),
                new OneTimeCommand(() -> robot.theOuttake.reset()),
                new MovementCommand(
                        sample1,
                        sample1Final,
                        defaultMovementConstants
                ),
                new YieldCommand(3000, robot.theIntake::containsSampleColorSensor),
                // new YieldCommand(250),
                new OneTimeCommand(() -> robot.theIntake.returnSlides()),
                new ParallelCommandGroup(
                        new MovementCommand(
                                sample1,
                                sample1Drop,
                                defaultMovementConstants
                        ),
                        new SequentialCommandGroup(
                                new YieldCommand(250),
                                new YieldCommand(robot.theOuttake::atTargetPosition),
                                new YieldCommand(robot.theIntake::linkageAtHome),
                                new OneTimeCommand(() -> robot.theOuttake.setSlidesState(Outtake.OuttakeSlidesStates.SAMPLES))
                        )
                ),
                new YieldCommand(2000, () -> robot.theOuttake.getSlidesState() == Outtake.OuttakeSlidesStates.SAMPLES),
                new YieldCommand(() -> robot.theOuttake.atTargetPosition()),
                new YieldCommand(600),
                new OneTimeCommand(() -> robot.theOuttake.setCurrentClawState(Outtake.OuttakeClawStates.DEFAULT)),
                new YieldCommand(100),
                new ParallelCommandGroup(
                        new MovementCommand(
                                sample1Drop,
                                sample2,
                                defaultMovementConstants
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
                // new YieldCommand(500),
                new OneTimeCommand(() -> robot.theIntake.returnSlides()),
                new ParallelCommandGroup(
                        new MovementCommand(
                                sample2,
                                sample2Drop,
                                defaultMovementConstants
                        ),
                        new SequentialCommandGroup(
                                new YieldCommand(250),
                                new YieldCommand(robot.theOuttake::atTargetPosition),
                                new YieldCommand(robot.theIntake::linkageAtHome),
                                new OneTimeCommand(() -> robot.theOuttake.setSlidesState(Outtake.OuttakeSlidesStates.SAMPLES))
                        )
                ),
                new YieldCommand(2000, () -> robot.theOuttake.getSlidesState() == Outtake.OuttakeSlidesStates.SAMPLES),
                new YieldCommand(() -> robot.theOuttake.atTargetPosition()),
                new YieldCommand(600),
                new OneTimeCommand(() -> robot.theOuttake.setCurrentClawState(Outtake.OuttakeClawStates.DEFAULT)),
                new YieldCommand(100),

                new ParallelCommandGroup(
                        new MovementCommand(
                                sample2Drop,
                                sample3,
                                new MovementConstants(0)
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
                new MovementCommand(
                        sample3,
                        sample3Final,
                        defaultMovementConstants
                ),
                new YieldCommand(3000, robot.theIntake::containsSampleColorSensor),
                // new YieldCommand(500),
                new OneTimeCommand(() -> robot.theIntake.returnSlides()),
                new ParallelCommandGroup(
                        new MovementCommand(
                                sample3,
                                sample3Drop,
                                defaultMovementConstants
                        ),
                        new SequentialCommandGroup(
                                new YieldCommand(250),
                                new YieldCommand(robot.theOuttake::atTargetPosition),
                                new YieldCommand(robot.theIntake::linkageAtHome),
                                new OneTimeCommand(() -> robot.theOuttake.setSlidesState(Outtake.OuttakeSlidesStates.SAMPLES))
                        )
                ),
                new YieldCommand(2000, () -> robot.theOuttake.getSlidesState() == Outtake.OuttakeSlidesStates.SAMPLES),
                new YieldCommand(() -> robot.theOuttake.atTargetPosition()),
                new YieldCommand(600),
                new OneTimeCommand(() -> robot.theOuttake.setCurrentClawState(Outtake.OuttakeClawStates.DEFAULT)),
                new YieldCommand(100),
                new ParallelCommandGroup(
                        new MovementCommand(
                                sample3Drop,
                                cycleInitial,
                                new MovementConstants(60, 60, 0, DriveConstants.K_V, DriveConstants.K_A)
                        ),
                        new SequentialCommandGroup(
                                new YieldCommand(100),
                                new OneTimeCommand(() -> robot.theOuttake.reset())
                        )
                ),
                new OneTimeCommand(() -> robot.theIntake.setTargetLinkageState(Intake.LinkageStates.AUTO_EXTENSION)),
                new MovementCommand(
                        cycleInitial,
                        cycleSubmersible,
                        new MovementConstants(0)
                ),
                new OneTimeCommand(() -> robot.theIntake.triggerCowcatcher()),
                new OneTimeCommand(() -> robot.theIntake.setDisableOuttake(false)),
                new YieldCommand(750),
                new OneTimeCommand(() -> robot.theIntake.setIntakeState(Intake.IntakeState.EXTENDED)),
                new OneTimeCommand(() -> robot.theIntake.setIntakeMotorState(Intake.IntakeMotorStates.INTAKING)),
                new YieldCommand(200),
                new OneTimeCommand(() -> robot.theIntake.setTargetLinkageState(Intake.LinkageStates.EXTENDED)),
                new MovementCommand(
                        cycleSubmersible,
                        cycleStrafe,
                        defaultMovementConstants

                ),
                new YieldCommand(2000, robot.theIntake::containsSampleColorSensor)
                // new YieldCommand(250)
                // new OneTimeCommand(() -> robot.theIntake.setIntakeState(Intake.IntakeState.EXTENDED)),
                // new OneTimeCommand(() -> robot.theIntake.setIntakeMotorState(Intake.IntakeMotorStates.INTAKING))

                /*
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
                */
        );

        SequentialCommandGroup cycleCommand = new SequentialCommandGroup(
                new OneTimeCommand(() -> robot.theIntake.setTargetLinkageState(Intake.LinkageStates.DEFAULT)),
                new OneTimeCommand(() -> robot.theIntake.returnSlides()),
                new MovementCommand(
                        cycleSubmersible,
                        cycleInitial,
                        new MovementConstants(60, 60, -0.1, DriveConstants.K_V, DriveConstants.K_A)
                ),
                new OneTimeCommand(() -> robot.theOuttake.setSlidesState(Outtake.OuttakeSlidesStates.SAMPLES)),
                new MovementCommand(
                        cycleInitial,
                        cycleDrop,
                        defaultMovementConstants
                ),
                new OneTimeCommand(() -> robot.theOuttake.setCurrentClawState(Outtake.OuttakeClawStates.DEFAULT)),
                new OneTimeCommand(() -> robot.theIntake.setTargetLinkageState(Intake.LinkageStates.AUTO_EXTENSION)),
                new YieldCommand(200),
                new ParallelCommandGroup(
                    new MovementCommand(
                            cycleDrop,
                            cycleInitial,
                            new MovementConstants(-0.2)
                    ),
                    new SequentialCommandGroup(
                            new YieldCommand(500),
                            new OneTimeCommand(() -> robot.theOuttake.reset())
                    )
                ),
                new OneTimeCommand(() -> robot.theIntake.setTargetLinkageState(Intake.LinkageStates.EXTENDED)),
                new MovementCommand(
                        cycleInitial,
                        cycleSubmersible,
                        defaultMovementConstants
                ),
                new OneTimeCommand(() -> robot.theIntake.setIntakeState(Intake.IntakeState.EXTENDED)),
                new OneTimeCommand(() -> robot.theIntake.setIntakeMotorState(Intake.IntakeMotorStates.INTAKING))

        );

        SequentialCommandGroup wrongColorCommand = new SequentialCommandGroup(
                new OneTimeCommand(() -> robot.theIntake.setTargetLinkageState(Intake.LinkageStates.AUTO_EXTENSION)),
                new OneTimeCommand(() -> robot.theIntake.reverseIntake()),
                new YieldCommand(250),
                new MovementCommand(
                        cycleSubmersible,
                        parkInitial,
                        new MovementConstants(50, 30, 0.1, DriveConstants.K_V, DriveConstants.K_A)
                ),
                new OneTimeCommand(() -> robot.theIntake.returnSlides()),
                new MovementCommand(
                        parkInitial,
                        parkFinal,
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

            if (gamepad1.triangle) {
                Globals.ALLIANCE = Alliance.RED;
            } else if (gamepad1.square) {
                Globals.ALLIANCE = Alliance.BLUE;

            }

            telemetry.addData("Alliance: ", Globals.ALLIANCE);
            telemetry.update();

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

        robot.theCommandScheduler.scheduleCommand(commands);

        while (!isStopRequested()) {

            if (!doneWithInitial) {
                if (commands.isFinished()) {
                    doneWithInitial = true;

                    if (robot.theIntake.sampleContained == Intake.SampleContained.BLUE && Globals.ALLIANCE == Alliance.RED) {
                        robot.theCommandScheduler.scheduleCommand(wrongColorCommand);
                    } else if (robot.theIntake.sampleContained == Intake.SampleContained.RED && Globals.ALLIANCE == Alliance.BLUE) {
                        robot.theCommandScheduler.scheduleCommand(wrongColorCommand);
                    } else if (robot.theIntake.sampleContained == Intake.SampleContained.NONE) {
                        robot.theCommandScheduler.scheduleCommand(wrongColorCommand);
                    } else {
                        robot.theCommandScheduler.scheduleCommand(cycleCommand);
                    }
                }
            }

            telemetry.addData("Sample contained: ", robot.theIntake.sampleContained);
            robot.update();
        }





    }
}
