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
import org.firstinspires.ftc.teamcode.utilities.robot.command.framework.commandtypes.DeadlineCommand;
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

    public static Pose startPose = new Pose(-37.1, -61.0, Math.PI / 2);
    public static Pose initialDrop = new Pose(-59.8, -52.2, 1.1784);
    public static Pose sample1 = new Pose(-58.2995, -51.5757, 1.1784);
    public static Pose sample1Final = new Pose(-57, -47.3757, 1.1784);
    public static Pose sample1Drop = new Pose(-58.1997, -51.2757, 1.1784);
    public static Pose sample2 = new Pose(-58.5219, -47.5666, 1.5987);
    public static Pose sample2Drop = new Pose(-58.2771, -52.8033, 1.1448);
    public static Pose sample3 = new Pose(-52.7078, -45.9624, 2.1477);
    public static Pose sample3Final = new Pose(-52.6078, -45.8624, 2.2477);
    public static Pose sample3Drop = new Pose(-58.1774, -52.9033, 1.1448);
    public static Pose partnerSamplePickupInitial = new Pose(-26.163, -57.8, 0);
    public static Pose partnerSamplePickupFinal = new Pose(-17.163, -57.8, 0);
    public static Pose cycleInitial = new Pose(-44, -9, 0);
    public static Pose cycleSubmersible = new Pose(-23, -9, 0);
    public static Pose cycleSubmersible2 = new Pose(-23, -3, 0);
    public static Pose cycleSubmersible3 = new Pose(-27, 3, 0.5);
    public static Pose cycleDrop = new Pose(-55, -56.5, Math.PI / 4);
    public static Pose cycleStrafe = new Pose(-23, -12, 0);
    public static Pose parkInitial = new Pose(-40, -9, Math.PI);
    public static Pose parkFinal = new Pose(-17, -9, Math.PI);

    public static MovementConstants defaultMovementConstants = new MovementConstants();

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.setMsTransmissionInterval(500);

        // Initialize the robot
        robot.init(this, telemetry);

        robot.theIntake.setDisableOuttake(true);

        boolean doneWithInitial = false;
        boolean doneWithPreloads = false;
        boolean submersibleCycle = true;
        boolean submersibleCycleDone = false;

        SequentialCommandGroup preloadedSamples = new SequentialCommandGroup(
                new OneTimeCommand(() -> robot.theOuttake.setSlidesState(Outtake.OuttakeSlidesStates.SAMPLES)),
                new OneTimeCommand(() -> robot.theIntake.setTargetLinkageState(Intake.LinkageStates.EXTENDED)),
                new MovementCommand(
                        startPose,
                        initialDrop,
                        new MovementConstants(0)
                ),
                new OneTimeCommand(() -> robot.theOuttake.setCurrentClawState(Outtake.OuttakeClawStates.DEFAULT)),
                new ParallelCommandGroup(
                        new MovementCommand(
                                initialDrop,
                                sample1,
                                new MovementConstants(-0.2)
                        )
                ),
                new OneTimeCommand(() -> robot.theIntake.setIntakeState(Intake.IntakeState.EXTENDED)),
                new OneTimeCommand(() -> robot.theIntake.setIntakeMotorState(Intake.IntakeMotorStates.INTAKING)),
                new OneTimeCommand(() -> robot.theOuttake.reset()),
                new DeadlineCommand(
                        new YieldCommand(robot.theIntake::containsSampleColorSensor),
                        new MovementCommand(
                                sample1,
                                sample1Final,
                                new MovementConstants(0)
                        )
                ),
                new YieldCommand(1000, robot.theIntake::containsSampleColorSensor),
                // new YieldCommand(250),
                new OneTimeCommand(() -> robot.theIntake.returnSlides()),
                new ParallelCommandGroup(
                        new MovementCommand(
                                sample1,
                                sample1Drop,
                                new MovementConstants(1)
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
                new YieldCommand(400),
                new OneTimeCommand(() -> robot.theOuttake.setCurrentClawState(Outtake.OuttakeClawStates.DEFAULT)),
                new OneTimeCommand(() -> robot.theIntake.setTargetLinkageState(Intake.LinkageStates.AUTO_EXTENSION)),
                new YieldCommand(50),
                new ParallelCommandGroup(
                        new MovementCommand(
                                sample1Drop,
                                sample2,
                                new MovementConstants(0)
                        ),
                        new SequentialCommandGroup(
                                new YieldCommand(750),
                                new OneTimeCommand(() -> robot.theIntake.setTargetLinkageState(Intake.LinkageStates.EXTENDED)),
                                new OneTimeCommand(() -> robot.theIntake.setIntakeState(Intake.IntakeState.EXTENDED)),
                                new OneTimeCommand(() -> robot.theIntake.setIntakeMotorState(Intake.IntakeMotorStates.INTAKING))
                        ),
                        new SequentialCommandGroup(
                                new YieldCommand(100),
                                new OneTimeCommand(() -> robot.theOuttake.reset())
                        )
                ),
                new YieldCommand(1000, robot.theIntake::containsSampleColorSensor),
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
                new YieldCommand(400),
                new OneTimeCommand(() -> robot.theOuttake.setCurrentClawState(Outtake.OuttakeClawStates.DEFAULT)),
                new YieldCommand(50),
                new OneTimeCommand(() -> robot.theIntake.setTargetLinkageState(Intake.LinkageStates.AUTO_EXTENSION)),
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
                new YieldCommand(1000, robot.theIntake::containsSampleColorSensor),
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
                new YieldCommand(400),
                new OneTimeCommand(() -> robot.theOuttake.setCurrentClawState(Outtake.OuttakeClawStates.DEFAULT)),
                new YieldCommand(50)
        );

        SequentialCommandGroup partnerSamplePickup = new SequentialCommandGroup(
                new ParallelCommandGroup(
                        new MovementCommand(
                                new Pose(sample3Drop.theX, sample3Drop.theY, partnerSamplePickupInitial.theHeading),
                                partnerSamplePickupInitial,
                                new MovementConstants(-0.3)
                        ),
                        new SequentialCommandGroup(
                                new YieldCommand(250),
                                new OneTimeCommand(() -> robot.theOuttake.reset()),
                                new OneTimeCommand(() -> robot.theIntake.setTargetLinkageState(Intake.LinkageStates.EXTENDED)),
                                new OneTimeCommand(() -> robot.theIntake.setIntakeState(Intake.IntakeState.EXTENDED)),
                                new OneTimeCommand(() -> robot.theIntake.setIntakeMotorState(Intake.IntakeMotorStates.INTAKING))

                        )
                ),
                new MovementCommand(
                        partnerSamplePickupInitial,
                        partnerSamplePickupFinal,
                        new MovementConstants(0)
                ),
                new YieldCommand(1000, robot.theIntake::containsSampleColorSensor),
                new YieldCommand(250),
                new OneTimeCommand(() -> robot.theIntake.returnSlides()),
                new OneTimeCommand(() -> robot.theOuttake.setSlidesState(Outtake.OuttakeSlidesStates.SAMPLES)),
                new ParallelCommandGroup(
                        new MovementCommand(
                                partnerSamplePickupFinal,
                                sample3Drop,
                                new MovementConstants(0)
                        )
                ),
                new YieldCommand(() -> robot.theOuttake.atTargetPosition()),
                new YieldCommand(400),
                new OneTimeCommand(() -> robot.theOuttake.setCurrentClawState(Outtake.OuttakeClawStates.DEFAULT)),
                new YieldCommand(50)
        );

        SequentialCommandGroup initialCycleCommand = new SequentialCommandGroup(
                new ParallelCommandGroup(
                        new MovementCommand(
                                sample3Drop,
                                cycleInitial,
                                new MovementConstants(80, 80, -0.4, DriveConstants.K_V, DriveConstants.K_A)
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
                        new MovementConstants(40, 40, 0, DriveConstants.K_V, DriveConstants.K_A)
                ),
                new OneTimeCommand(() -> robot.theIntake.triggerCowcatcher()),
                new YieldCommand(550),
                new OneTimeCommand(() -> robot.theIntake.setIntakeState(Intake.IntakeState.EXTENDED)),
                new OneTimeCommand(() -> robot.theIntake.setIntakeMotorState(Intake.IntakeMotorStates.INTAKING)),
                new YieldCommand(50),
                new OneTimeCommand(() -> robot.theIntake.setTargetLinkageState(Intake.LinkageStates.EXTENDED)),
                new DeadlineCommand(
                        new YieldCommand(robot.theIntake::containsSampleColorSensor),
                        new SequentialCommandGroup(
                                new MovementCommand(
                                        cycleSubmersible,
                                        cycleStrafe,
                                        new MovementConstants(0.2)
                                ),
                                new MovementCommand(
                                        cycleStrafe,
                                        cycleSubmersible2,
                                        new MovementConstants(-0.25)
                                ),
                                new MovementCommand(
                                        cycleSubmersible2,
                                        cycleSubmersible3,
                                        new MovementConstants(0.5)
                                )
                        )
                ),
                new YieldCommand(100)
        );

        SequentialCommandGroup cycleCommand = new SequentialCommandGroup(
                new OneTimeCommand(() -> robot.theIntake.returnSlides()),
                new OneTimeCommand(() -> robot.theIntake.setCurrentCowcatcherState(Intake.CowcatcherStates.ACTIVATED)),
                new MovementCommand(
                        cycleSubmersible,
                        cycleInitial,
                        new MovementConstants(80, 80, 0, DriveConstants.K_V, DriveConstants.K_A)
                ),
                new OneTimeCommand(() -> robot.theOuttake.setSlidesState(Outtake.OuttakeSlidesStates.SAMPLES)),
                new MovementCommand(
                        cycleInitial,
                        cycleDrop,
                        new MovementConstants(0.1)
                ),
                new OneTimeCommand(() -> robot.theOuttake.setCurrentClawState(Outtake.OuttakeClawStates.DEFAULT)),
                new OneTimeCommand(() -> robot.theIntake.setCurrentCowcatcherState(Intake.CowcatcherStates.DEFAULT)),
                new YieldCommand(100),
                new ParallelCommandGroup(
                    new MovementCommand(
                            cycleDrop,
                            cycleInitial,
                            new MovementConstants(80, 80, -0.25, DriveConstants.K_V, DriveConstants.K_A)
                    ),
                    new SequentialCommandGroup(
                            new YieldCommand(500),
                            new OneTimeCommand(() -> robot.theOuttake.reset())
                    )
                ),
                new OneTimeCommand(() -> robot.theIntake.setTargetLinkageState(Intake.LinkageStates.AUTO_EXTENSION)),
                new MovementCommand(
                        cycleInitial,
                        cycleSubmersible,
                        new MovementConstants(20, 40, -0.25, DriveConstants.K_V, DriveConstants.K_A)
                ),
                new OneTimeCommand(() -> robot.theIntake.setTargetLinkageState(Intake.LinkageStates.EXTENDED)),
                new OneTimeCommand(() -> robot.theIntake.setIntakeState(Intake.IntakeState.EXTENDED)),
                new OneTimeCommand(() -> robot.theIntake.setIntakeMotorState(Intake.IntakeMotorStates.INTAKING))
        );

        SequentialCommandGroup initialCycleCommand2 = new SequentialCommandGroup(
                new DeadlineCommand(
                        new YieldCommand(robot.theIntake::containsSampleColorSensor),
                        new SequentialCommandGroup(
                                new MovementCommand(
                                        cycleStrafe,
                                        cycleSubmersible2,
                                        new MovementConstants(-0.25)
                                ),
                                new MovementCommand(
                                        cycleSubmersible2,
                                        cycleSubmersible3,
                                        new MovementConstants(0.5)
                                )
                        )
                ),
                new YieldCommand(100)
        );

        SequentialCommandGroup cycleCommand2 = new SequentialCommandGroup(
                new OneTimeCommand(() -> robot.theIntake.returnSlides()),
                new OneTimeCommand(() -> robot.theIntake.setCurrentCowcatcherState(Intake.CowcatcherStates.ACTIVATED)),
                new MovementCommand(
                        cycleSubmersible,
                        cycleInitial,
                        new MovementConstants(80, 80, 0, DriveConstants.K_V, DriveConstants.K_A)
                ),
                new OneTimeCommand(() -> robot.theOuttake.setSlidesState(Outtake.OuttakeSlidesStates.SAMPLES)),
                new MovementCommand(
                        cycleInitial,
                        cycleDrop,
                        new MovementConstants(40, 40, 0.25, DriveConstants.K_V, DriveConstants.K_A)
                ),
                new OneTimeCommand(() -> robot.theOuttake.setCurrentClawState(Outtake.OuttakeClawStates.DEFAULT)),
                new OneTimeCommand(() -> robot.theIntake.setCurrentCowcatcherState(Intake.CowcatcherStates.DEFAULT)),
                new YieldCommand(100),
                new ParallelCommandGroup(
                        new MovementCommand(
                                cycleDrop,
                                cycleInitial,
                                new MovementConstants(80, 80, -0.3, DriveConstants.K_V, DriveConstants.K_A)
                        ),
                        new SequentialCommandGroup(
                                new YieldCommand(500),
                                new OneTimeCommand(() -> robot.theOuttake.reset())
                        )
                ),
                new OneTimeCommand(() -> robot.theIntake.setTargetLinkageState(Intake.LinkageStates.AUTO_EXTENSION)),
                new MovementCommand(
                        cycleInitial,
                        cycleSubmersible,
                        defaultMovementConstants
                ),
                new OneTimeCommand(() -> robot.theIntake.setTargetLinkageState(Intake.LinkageStates.EXTENDED)),
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
                        new MovementConstants(50, 50, -0.2, DriveConstants.K_V, DriveConstants.K_A)
                ),
                new OneTimeCommand(() -> robot.theIntake.returnSlides()),
                new MovementCommand(
                        parkInitial,
                        parkFinal,
                        new MovementConstants(50, 40, 0, DriveConstants.K_V, DriveConstants.K_A)
                ),
                new OneTimeCommand(() -> robot.theIntake.setTargetLinkageState(Intake.LinkageStates.EXTENDED)),
                new OneTimeCommand(() -> robot.theOuttake.setCurrentOuttakeState(Outtake.OuttakeServoState.AUTO_PARK))
        );

        SequentialCommandGroup wrongColorCommand2 = new SequentialCommandGroup(
                new OneTimeCommand(() -> robot.theIntake.setTargetLinkageState(Intake.LinkageStates.AUTO_EXTENSION)),
                new OneTimeCommand(() -> robot.theIntake.reverseIntake()),
                new YieldCommand(250),
                new MovementCommand(
                        cycleSubmersible,
                        parkInitial,
                        new MovementConstants(50, 50, -0.2, DriveConstants.K_V, DriveConstants.K_A)
                ),
                new OneTimeCommand(() -> robot.theIntake.returnSlides()),
                new MovementCommand(
                        parkInitial,
                        parkFinal,
                        new MovementConstants(50, 40, 0, DriveConstants.K_V, DriveConstants.K_A)
                ),
                new OneTimeCommand(() -> robot.theIntake.setTargetLinkageState(Intake.LinkageStates.EXTENDED)),
                new OneTimeCommand(() -> robot.theOuttake.setCurrentOuttakeState(Outtake.OuttakeServoState.AUTO_PARK))
        );

        robot.theOuttake.setCurrentClawState(Outtake.OuttakeClawStates.CLOSED);
        robot.theIntake.leftServo.setPosition(Intake.LinkageStates.DEFAULT.position - 0.03);
        robot.theIntake.rightServo.setPosition(Intake.LinkageStates.DEFAULT.position - 0.03);

        while (opModeInInit()) {
            if (gamepad1.cross) {
                robot.theOuttake.clawServo.setPosition(Outtake.OuttakeClawStates.CLOSED.position);
            }

            if (gamepad1.circle) {
                robot.theIntake.leftDropdownServo.setPosition(Intake.IntakeState.AUTO_DEFAULT.position);
                robot.theIntake.rightDropdownServo.setPosition(Intake.IntakeState.AUTO_DEFAULT.position);

            }

            if (gamepad1.triangle) {
                Globals.ALLIANCE = Alliance.RED;
            } else if (gamepad1.square) {
                Globals.ALLIANCE = Alliance.BLUE;
            }

            if (gamepad2.cross) {
                submersibleCycle = false;
            } else if (gamepad2.square) {
                submersibleCycle = true;
            }

            telemetry.addLine("Press cross (gamepad1) to close claw");
            telemetry.addLine("Press circle (gamepad1) to set intake dropdown servos to AUTO_DEFAULT");
            telemetry.addLine("Press triangle (gamepad1) to set alliance to RED");
            telemetry.addLine("Press square (gamepad1) to set alliance to BLUE");
            telemetry.addLine("Press cross (gamepad2) to disable submersible cycle");
            telemetry.addLine("Press square (gamepad2) to enable submersible cycle");


            telemetry.addData("Alliance: ", Globals.ALLIANCE);
            telemetry.addData("Submersible Cycle: ", submersibleCycle);

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

        robot.update();

        robot.theCommandScheduler.scheduleCommand(preloadedSamples);

        while (!isStopRequested()) {


            if (!doneWithPreloads) {
                if (preloadedSamples.isFinished()) {
                    doneWithPreloads = true;

                    if (!submersibleCycle) {
                        robot.theCommandScheduler.scheduleCommand(
                                new SequentialCommandGroup(partnerSamplePickup, initialCycleCommand)
                        );
                    } else {
                        robot.theCommandScheduler.scheduleCommand(initialCycleCommand);
                    }
                }
            }


            if (!doneWithInitial) {
                if (preloadedSamples.isFinished() && initialCycleCommand.isFinished()) {
                    doneWithInitial = true;

                    robot.theIntake.updatePossessedColor();

                    System.out.println(robot.theIntake.sampleContained);

                    if (robot.theIntake.sampleContained == Intake.SampleContained.BLUE && Globals.ALLIANCE == Alliance.RED) {
                        robot.theCommandScheduler.scheduleCommand(wrongColorCommand);
                    } else if (robot.theIntake.sampleContained == Intake.SampleContained.RED && Globals.ALLIANCE == Alliance.BLUE) {
                        robot.theCommandScheduler.scheduleCommand(wrongColorCommand);
                    } else if (robot.theIntake.sampleContained == Intake.SampleContained.NONE && !robot.theIntake.containsSampleColorSensor()) {
                        robot.theCommandScheduler.scheduleCommand(wrongColorCommand);
                    } else {
                        robot.theCommandScheduler.scheduleCommand(
                                new SequentialCommandGroup(
                                        cycleCommand,
                                        initialCycleCommand2
                                )
                        );
                    }
                }
            }

            if (!submersibleCycleDone && submersibleCycle) {
                if (initialCycleCommand.isFinished() && cycleCommand.isFinished() && initialCycleCommand2.isFinished()) {
                    submersibleCycleDone = true;
                    robot.theIntake.updatePossessedColor();

                    System.out.println(robot.theIntake.sampleContained);

                    if (robot.theIntake.sampleContained == Intake.SampleContained.BLUE && Globals.ALLIANCE == Alliance.RED) {
                        robot.theCommandScheduler.scheduleCommand(wrongColorCommand2);
                    } else if (robot.theIntake.sampleContained == Intake.SampleContained.RED && Globals.ALLIANCE == Alliance.BLUE) {
                        robot.theCommandScheduler.scheduleCommand(wrongColorCommand2);
                    } else if (robot.theIntake.sampleContained == Intake.SampleContained.NONE && !robot.theIntake.containsSampleColorSensor()) {
                        robot.theCommandScheduler.scheduleCommand(wrongColorCommand2);
                    } else {
                        System.out.println("Scheduling cycle command 2");
                        robot.theCommandScheduler.scheduleCommand(
                                new SequentialCommandGroup(
                                        cycleCommand2
                                )
                        );
                    }
                }
            }

            telemetry.addData("Sample contained: ", robot.theIntake.sampleContained);
            robot.update();
        }





    }
}
