package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.utilities.math.linearalgebra.Pose;
import org.firstinspires.ftc.teamcode.utilities.robot.Alliance;
import org.firstinspires.ftc.teamcode.utilities.robot.Globals;
import org.firstinspires.ftc.teamcode.utilities.robot.RobotEx;
import org.firstinspires.ftc.teamcode.utilities.robot.command.framework.commandtypes.DeadlineCommand;
import org.firstinspires.ftc.teamcode.utilities.robot.command.framework.commandtypes.OneTimeCommand;
import org.firstinspires.ftc.teamcode.utilities.robot.command.framework.commandtypes.ParallelCommandGroup;
import org.firstinspires.ftc.teamcode.utilities.robot.command.framework.commandtypes.SequentialCommandGroup;
import org.firstinspires.ftc.teamcode.utilities.robot.command.framework.commandtypes.YieldCommand;
import org.firstinspires.ftc.teamcode.utilities.robot.command.movement.InitialCycleCommand1;
import org.firstinspires.ftc.teamcode.utilities.robot.command.movement.MovementCommand;
import org.firstinspires.ftc.teamcode.utilities.robot.command.movement.RetryCommand;
import org.firstinspires.ftc.teamcode.utilities.robot.movement.MovementConstants;
import org.firstinspires.ftc.teamcode.utilities.robot.movement.PIDDrive;
import org.firstinspires.ftc.teamcode.utilities.robot.subsystems.Intake;
import org.firstinspires.ftc.teamcode.utilities.robot.subsystems.Outtake;

/**
 * Example teleop code for a basic mecanum drive
 */
/*
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
        public static Pose startPose = new MovementCommand(new Pose(9.3, -62.2, Math.PI / 2), new Pose(1.77, -32.1, Math.PI / 2), new MovementConstants());
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


                );,






        while (!isStopRequested()) {
            robot.update();
        }

    }
}


/**
 * Example teleop code for a basic mecanum drive
 */

@Autonomous(name = "Specimen Auto")
public class SpecimenAuto extends LinearOpMode {

    // Create new Instance of the robot
    RobotEx robot = RobotEx.getInstance();

    public static Pose startPose = new Pose(9.88, -61.1, Math.PI / 2);
    public static Pose Spec1 = new Pose(-4.33, -28.6, Math.PI / 2);
    public static Pose Spec1Drop = new Pose(-4.33, -38.6, Math.PI / 2);
    public static Pose Spec2Pickup = new Pose(21.7 - 2, -41.5 - 2, 0.5677);
    public static Pose Spec2PickupFinal = new Pose(21.7 + 4 * Math.cos(0.5677), -41.5 + 4 * Math.sin(0.5677), 0.5677);
    public static Pose Spec2Spit = new Pose(24.99, -43.72, -0.6267);
    public static Pose Spec3Pickup = new Pose(33.2 - 2, -41.3 - 2, 0.5844);
    public static Pose Spec3PickupFinal = new Pose(33.2 + 4 * Math.cos(0.5844), -41.3 + 4 * Math.sin(0.5844), 0.5844);

    public static Pose Spec3Spit = new Pose(37.47, -42.1, -0.809);
    public static Pose Spec4Pickup = new Pose(41.22 - 2, -40.1 - 2, 0.5275);
    public static Pose Spec4PickupFinal = new Pose(41.22 + 4 * Math.cos(0.5275), -40.1 + 4 * Math.sin(0.5275), 0.5275);

    public static Pose Spec4Spit = new Pose(36.57, -38.5, -0.8783);
    public static Pose SpecLoadInitial = new Pose(34.79, -50.93, Math.PI / 2);
    public static Pose SpecLoadInitial2 = new Pose(34.79, -63.93, Math.PI / 2);
    public static Pose SpecLoadFinal = new Pose(-4.33+2, -28.6, Math.PI / 2);
    public static Pose SpecLoadFinal2 = new Pose(-4.33+2, -35.6, Math.PI / 2);
    public static Pose Spec3LoadFinal = new Pose(-4.33+4, -28.6, Math.PI / 2);
    public static Pose Spec3LoadFinal2 = new Pose(-4.33+4, -35.6, Math.PI / 2);
    public static Pose Spec4LoadFinal = new Pose(-4.33+6, -28.6, Math.PI / 2);
    public static Pose Spec4LoadFinal2 = new Pose(-4.33+6, -35.6, Math.PI / 2);
    public static Pose Spec5LoadFinal = new Pose(-4.33+8, -28.6, Math.PI / 2);
    public static Pose Spec5LoadFinal2 = new Pose(-4.33+8, -35.6, Math.PI / 2);
    public static Pose Park = new Pose(-4.33+2, -58, Math.PI / 2);
    public static Pose Spec2PlaceInitial = new Pose(28, 6, 0);
    public static Pose Spec2PlaceFinal = new Pose(32.4, 6, 0);
    public static Pose Spec3PlaceInitial = new Pose(28, 8, 0);
    public static Pose Spec3PlaceFinal = new Pose(32.4, 8, 0);
    public static Pose Spec4PlaceInitial = new Pose(28, 10, 0);
    public static Pose Spec4PlaceFinal = new Pose(32.4, 10, 0);
    public static Pose Spec5PlaceInitial = new Pose(28, 12, 0);
    public static Pose Spec5PlaceFinal = new Pose(32.4, 12, 0);
    public static Pose parkFinal = new Pose(0, -25.87, 0);

    public static MovementConstants defaultMovementConstants = new MovementConstants();
    public static MovementConstants scoreEndMovementConstant = new MovementConstants(100, 100, -0.5);
    public static MovementConstants scoreApproachMovementConstant = new MovementConstants(50, 100, -0.4);
    public static MovementConstants pickupApproachMovementConstant = new MovementConstants(60, 100, -0.1);
    public static MovementConstants pickupEndMovementConstant = new MovementConstants(20, 100, 0);

    public static long placeWait = 150; // in ms
    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.setMsTransmissionInterval(500);

        // Initialize the robot
        robot.init(this, telemetry);


        SequentialCommandGroup preloadedSpecimen = new SequentialCommandGroup(


                new SequentialCommandGroup(
                        new ParallelCommandGroup(
                                new OneTimeCommand(() -> robot.theIntake.setIntakeState(Intake.IntakeState.AUTO_DEFAULT)),
                                new OneTimeCommand(() -> robot.theOuttake.setCurrentClawState(Outtake.OuttakeClawStates.CLOSED)),
                                new OneTimeCommand(() -> robot.theOuttake.setSlidesState(Outtake.OuttakeSlidesStates.SPECIMENS)),
                                new OneTimeCommand(() -> robot.theOuttake.setCurrentOuttakeState(Outtake.OuttakeServoState.SPECIMEN_INITIAL)),
                                new OneTimeCommand(() -> robot.theOuttake.setCurrentPivotState(Outtake.OuttakePivotStates.SPECIMEN_DROP)),
                                new OneTimeCommand(() -> robot.theOuttake.setCurrentRotationState(Outtake.OuttakeRotationStates.SPECIMEN_ROTATED))
                        ),
                        new MovementCommand(
                                startPose,
                                Spec1,
                                new MovementConstants(-0.5)
                        ),
                        new OneTimeCommand(() -> robot.theOuttake.setSlidesState(Outtake.OuttakeSlidesStates.SPECIMENS_DROP)),
                        new OneTimeCommand(() -> robot.theOuttake.setCurrentClawState(Outtake.OuttakeClawStates.LESS_CLOSED)),
                        new YieldCommand(150),
                        new MovementCommand(
                                Spec1,
                                Spec1Drop,
                                scoreEndMovementConstant
                        ),
                        new ParallelCommandGroup(
                                new MovementCommand(
                                        Spec1Drop,
                                        Spec2Pickup,
                                        new MovementConstants(-0.5)
                                ),
                                new SequentialCommandGroup(
                                        new YieldCommand(700),
                                        new OneTimeCommand(() -> robot.theIntake.setTargetLinkageState(Intake.LinkageStates.EXTENDED))
                                )
                        ),
                        // start cycle
                        new OneTimeCommand(() -> robot.theIntake.setIntakeState(Intake.IntakeState.EXTENDED)),
                        new OneTimeCommand(() -> robot.theIntake.setIntakeMotorState(Intake.IntakeMotorStates.INTAKING)),
                        new DeadlineCommand(
                                new YieldCommand(600, robot.theIntake::containsSampleColorSensor),
                                new SequentialCommandGroup(
                                        new MovementCommand(
                                                Spec2Pickup,
                                                Spec2PickupFinal,
                                                new MovementConstants(2)
                                        )
                                )
                        ),
                        new OneTimeCommand(() -> robot.theIntake.setTargetHolderState(Intake.SampleHolderState.EXTENDED)),
                        new OneTimeCommand(() -> robot.theIntake.setIntakeState(Intake.IntakeState.DEFAULT)),
                        new OneTimeCommand(() -> robot.theIntake.setTargetLinkageState(Intake.LinkageStates.AUTO_EXTENSION_FURTHER)),
                        new MovementCommand(
                                Spec2Pickup,
                                Spec2Spit,
                                new MovementConstants(-0.2)
                        ),
                        new OneTimeCommand(() -> robot.theIntake.setTargetHolderState(Intake.SampleHolderState.DEFAULT)),
                        new OneTimeCommand(() -> robot.theIntake.setIntakeMotorState(Intake.IntakeMotorStates.REVERSE)),

                        new YieldCommand(500),
                        // end spit


                        new MovementCommand(
                                Spec2Spit,
                                Spec3Pickup,
                                new MovementConstants(-0.2)
                        ),
                        new OneTimeCommand(() -> robot.theIntake.setTargetLinkageState(Intake.LinkageStates.EXTENDED)),
                        new OneTimeCommand(() -> robot.theIntake.setIntakeState(Intake.IntakeState.EXTENDED)),
                        new OneTimeCommand(() -> robot.theIntake.setIntakeMotorState(Intake.IntakeMotorStates.INTAKING)),

                        new DeadlineCommand(
                                new YieldCommand(600, robot.theIntake::containsSampleColorSensor),
                                new SequentialCommandGroup(
                                        new MovementCommand(
                                                Spec3Pickup,
                                                Spec3PickupFinal,
                                                new MovementConstants(2)
                                        )
                                )
                        ),
                        new OneTimeCommand(() -> robot.theIntake.setTargetHolderState(Intake.SampleHolderState.EXTENDED)),
                        new OneTimeCommand(() -> robot.theIntake.setIntakeState(Intake.IntakeState.DEFAULT)),
                        new OneTimeCommand(() -> robot.theIntake.setTargetLinkageState(Intake.LinkageStates.AUTO_EXTENSION_FURTHER)),

                        new MovementCommand(
                                Spec3Pickup,
                                Spec3Spit,
                                new MovementConstants(-0.2)
                        ),
                        new OneTimeCommand(() -> robot.theIntake.setTargetHolderState(Intake.SampleHolderState.DEFAULT)),
                        new OneTimeCommand(() -> robot.theIntake.setIntakeMotorState(Intake.IntakeMotorStates.REVERSE)),

                        new YieldCommand(500),
                        // end spit
                        new MovementCommand(
                                Spec3Spit,
                                Spec4Pickup,
                                new MovementConstants(0.2)
                        ),
                        new OneTimeCommand(() -> robot.theIntake.setTargetLinkageState(Intake.LinkageStates.EXTENDED)),
                        new OneTimeCommand(() -> robot.theIntake.setIntakeState(Intake.IntakeState.EXTENDED)),
                        new OneTimeCommand(() -> robot.theIntake.setIntakeMotorState(Intake.IntakeMotorStates.INTAKING)),

                        new DeadlineCommand(
                                new YieldCommand(600, robot.theIntake::containsSampleColorSensor),
                                new SequentialCommandGroup(
                                        new MovementCommand(
                                                Spec4Pickup,
                                                Spec4PickupFinal,
                                                new MovementConstants(2)
                                        )
                                )
                        ),
                        new OneTimeCommand(() -> robot.theIntake.setTargetHolderState(Intake.SampleHolderState.EXTENDED)),
                        new OneTimeCommand(() -> robot.theIntake.setIntakeState(Intake.IntakeState.DEFAULT)),
                        new OneTimeCommand(() -> robot.theIntake.setTargetLinkageState(Intake.LinkageStates.AUTO_EXTENSION_FURTHER)),

                        new MovementCommand(
                                Spec4Pickup,
                                Spec4Spit,
                                new MovementConstants(-0.2)
                        ),
                        new OneTimeCommand(() -> robot.theIntake.setTargetHolderState(Intake.SampleHolderState.DEFAULT)),
                        new OneTimeCommand(() -> robot.theIntake.setIntakeMotorState(Intake.IntakeMotorStates.REVERSE)),

                        new YieldCommand(500),
                        new OneTimeCommand(() -> robot.theIntake.setTargetLinkageState(Intake.LinkageStates.DEFAULT)),
                        new OneTimeCommand(() -> robot.theIntake.setIntakeMotorState(Intake.IntakeMotorStates.STATIONARY)),
                        // end spit
                        new ParallelCommandGroup(
                                new SequentialCommandGroup(
                                        new MovementCommand(
                                                Spec4Spit,
                                                SpecLoadInitial,
                                                pickupApproachMovementConstant
                                        ),
                                        new MovementCommand(
                                                SpecLoadInitial,
                                                SpecLoadInitial2,
                                                pickupEndMovementConstant
                                        )
                                ),
                                new SequentialCommandGroup(
                                        new OneTimeCommand(() -> robot.theOuttake.setSlidesState(Outtake.OuttakeSlidesStates.SPECIMEN_INITIAL_PICKUP)),
                                        new YieldCommand(200),
                                        new OneTimeCommand(() -> robot.theOuttake.setCurrentOuttakeState(Outtake.OuttakeServoState.SPECIMEN_PICKUP)),
                                        new OneTimeCommand(() -> robot.theOuttake.setCurrentPivotState(Outtake.OuttakePivotStates.SPECIMEN_PICKUP)),
                                        new OneTimeCommand(() -> robot.theOuttake.setCurrentClawState(Outtake.OuttakeClawStates.SPECIMEN_PICKUP)),
                                        new OneTimeCommand(() -> robot.theOuttake.setCurrentRotationState(Outtake.OuttakeRotationStates.ROTATED)),
                                        new YieldCommand(1000, robot.theOuttake::atTargetPosition),
                                        new YieldCommand(100),
                                        new OneTimeCommand(() -> robot.theOuttake.setSlidesState(Outtake.OuttakeSlidesStates.SPECIMEN_PICKUP))
                                )
                        ),
                        new OneTimeCommand(() -> robot.theOuttake.setCurrentClawState(Outtake.OuttakeClawStates.CLOSED)),
                        new YieldCommand(250),
                        new MovementCommand(
                                SpecLoadInitial2,
                                SpecLoadFinal,
                                scoreApproachMovementConstant
                        ),
                        new OneTimeCommand(() -> {
                            robot.theOuttake.setSlidesState(Outtake.OuttakeSlidesStates.SPECIMENS_DROP);
                            robot.theOuttake.setCurrentClawState(Outtake.OuttakeClawStates.LESS_CLOSED);
                        }),
                        new YieldCommand(placeWait),
                        new MovementCommand(
                                SpecLoadFinal,
                                SpecLoadFinal2,
                                scoreEndMovementConstant
                        ),
                        //new OneTimeCommand(() -> robot.theOuttake.setCurrentClawState(Outtake.OuttakeClawStates.DEFAULT)),
                        // new OneTimeCommand(() -> robot.theOuttake.setCurrentClawState(Outtake.OuttakeClawStates.DEFAULT)),
                        new ParallelCommandGroup(
                                new MovementCommand(
                                        SpecLoadFinal2,
                                        SpecLoadInitial,
                                        pickupApproachMovementConstant
                                ),
                                new SequentialCommandGroup(
                                        new YieldCommand(200),
                                        new OneTimeCommand(() -> robot.theOuttake.setSlidesState(Outtake.OuttakeSlidesStates.SPECIMEN_INITIAL_PICKUP)),
                                        new YieldCommand(200),
                                        new OneTimeCommand(() -> robot.theOuttake.setCurrentOuttakeState(Outtake.OuttakeServoState.SPECIMEN_PICKUP)),
                                        new OneTimeCommand(() -> robot.theOuttake.setCurrentPivotState(Outtake.OuttakePivotStates.SPECIMEN_PICKUP)),
                                        new OneTimeCommand(() -> robot.theOuttake.setCurrentClawState(Outtake.OuttakeClawStates.SPECIMEN_PICKUP)),
                                        new OneTimeCommand(() -> robot.theOuttake.setCurrentRotationState(Outtake.OuttakeRotationStates.ROTATED)),
                                        new YieldCommand(1000, robot.theOuttake::atTargetPosition),
                                        new YieldCommand(100),
                                        new OneTimeCommand(() -> robot.theOuttake.setSlidesState(Outtake.OuttakeSlidesStates.SPECIMEN_PICKUP))
                                )
                        ),
                        new MovementCommand(
                                SpecLoadInitial,
                                SpecLoadInitial2,
                                pickupEndMovementConstant
                        ),
                        // End copy
                        new OneTimeCommand(() -> robot.theOuttake.setCurrentClawState(Outtake.OuttakeClawStates.CLOSED)),
                        new YieldCommand(250),
                        new MovementCommand(
                                SpecLoadInitial2,
                                Spec3LoadFinal,
                                scoreApproachMovementConstant
                        ),
                        new OneTimeCommand(() -> {
                            robot.theOuttake.setSlidesState(Outtake.OuttakeSlidesStates.SPECIMENS_DROP);
                            robot.theOuttake.setCurrentClawState(Outtake.OuttakeClawStates.LESS_CLOSED);
                        }),
                        new YieldCommand(placeWait),
                        new MovementCommand(
                                Spec3LoadFinal,
                                Spec3LoadFinal2,
                                scoreEndMovementConstant
                        ),
                       // new OneTimeCommand(() -> robot.theOuttake.setCurrentClawState(Outtake.OuttakeClawStates.DEFAULT)),
                        // new OneTimeCommand(() -> robot.theOuttake.setCurrentClawState(Outtake.OuttakeClawStates.DEFAULT)),
                        new ParallelCommandGroup(
                                new MovementCommand(
                                        Spec3LoadFinal2,
                                        SpecLoadInitial,
                                        pickupApproachMovementConstant
                                ),
                                new SequentialCommandGroup(
                                        new YieldCommand(200),
                                        new OneTimeCommand(() -> robot.theOuttake.setSlidesState(Outtake.OuttakeSlidesStates.SPECIMEN_INITIAL_PICKUP)),
                                        new YieldCommand(200),
                                        new OneTimeCommand(() -> robot.theOuttake.setCurrentOuttakeState(Outtake.OuttakeServoState.SPECIMEN_PICKUP)),
                                        new OneTimeCommand(() -> robot.theOuttake.setCurrentPivotState(Outtake.OuttakePivotStates.SPECIMEN_PICKUP)),
                                        new OneTimeCommand(() -> robot.theOuttake.setCurrentClawState(Outtake.OuttakeClawStates.SPECIMEN_PICKUP)),
                                        new OneTimeCommand(() -> robot.theOuttake.setCurrentRotationState(Outtake.OuttakeRotationStates.ROTATED)),
                                        new YieldCommand(1000, robot.theOuttake::atTargetPosition),
                                        new YieldCommand(100),
                                        new OneTimeCommand(() -> robot.theOuttake.setSlidesState(Outtake.OuttakeSlidesStates.SPECIMEN_PICKUP))
                                )
                        ),
                        new MovementCommand(
                                SpecLoadInitial,
                                SpecLoadInitial2,
                                pickupEndMovementConstant
                        ),
                        new OneTimeCommand(() -> robot.theOuttake.setCurrentClawState(Outtake.OuttakeClawStates.CLOSED)),
                        new YieldCommand(250),
                        new MovementCommand(
                                SpecLoadInitial2,
                                Spec4LoadFinal,
                                scoreApproachMovementConstant
                        ),
                        new OneTimeCommand(() -> {
                            robot.theOuttake.setSlidesState(Outtake.OuttakeSlidesStates.SPECIMENS_DROP);
                            robot.theOuttake.setCurrentClawState(Outtake.OuttakeClawStates.LESS_CLOSED);
                        }),
                        new YieldCommand(placeWait),

                        new MovementCommand(
                                Spec4LoadFinal,
                                Spec4LoadFinal2,
                                scoreEndMovementConstant
                        ),
                        // new OneTimeCommand(() -> robot.theOuttake.setCurrentClawState(Outtake.OuttakeClawStates.DEFAULT)),
                        new ParallelCommandGroup(
                                new MovementCommand(
                                        Spec4LoadFinal2,
                                        SpecLoadInitial,
                                        pickupApproachMovementConstant
                                ),
                                new SequentialCommandGroup(
                                        new YieldCommand(200),
                                        new OneTimeCommand(() -> robot.theOuttake.setSlidesState(Outtake.OuttakeSlidesStates.SPECIMEN_INITIAL_PICKUP)),
                                        new YieldCommand(200),
                                        new OneTimeCommand(() -> robot.theOuttake.setCurrentOuttakeState(Outtake.OuttakeServoState.SPECIMEN_PICKUP)),
                                        new OneTimeCommand(() -> robot.theOuttake.setCurrentPivotState(Outtake.OuttakePivotStates.SPECIMEN_PICKUP)),
                                        new OneTimeCommand(() -> robot.theOuttake.setCurrentClawState(Outtake.OuttakeClawStates.SPECIMEN_PICKUP)),
                                        new OneTimeCommand(() -> robot.theOuttake.setCurrentRotationState(Outtake.OuttakeRotationStates.ROTATED)),
                                        new YieldCommand(1000, robot.theOuttake::atTargetPosition),
                                        new YieldCommand(100),
                                        new OneTimeCommand(() -> robot.theOuttake.setSlidesState(Outtake.OuttakeSlidesStates.SPECIMEN_PICKUP))
                                )
                        ),
                        new MovementCommand(
                                SpecLoadInitial,
                                SpecLoadInitial2,
                                pickupEndMovementConstant
                        ),
                        new OneTimeCommand(() -> robot.theOuttake.setCurrentClawState(Outtake.OuttakeClawStates.CLOSED)),
                        new YieldCommand(250),
                        new MovementCommand(
                                SpecLoadInitial2,
                                Spec5LoadFinal,
                                scoreApproachMovementConstant
                        ),
                        new OneTimeCommand(() -> {
                            robot.theOuttake.setSlidesState(Outtake.OuttakeSlidesStates.SPECIMENS_DROP);
                            robot.theOuttake.setCurrentClawState(Outtake.OuttakeClawStates.LESS_CLOSED);
                        }),
                        new YieldCommand(placeWait),

                        new MovementCommand(
                                Spec5LoadFinal,
                                Spec5LoadFinal2,
                                scoreEndMovementConstant
                        ),
                        new MovementCommand(
                                Spec5LoadFinal2,
                                Park,
                                new MovementConstants(-0.5)
                        )
                        // new OneTimeCommand(() -> robot.theOuttake.setCurrentClawState(Outtake.OuttakeClawStates.DEFAULT))
                        /*
                        new ParallelCommandGroup(
                                new MovementCommand(
                                        Spec5LoadFinal2,
                                        SpecLoadInitial,
                                        new MovementConstants(0.2)
                                ),
                                new SequentialCommandGroup(
                                        new YieldCommand(200),
                                        new OneTimeCommand(() -> robot.theOuttake.setSlidesState(Outtake.OuttakeSlidesStates.SPECIMEN_INITIAL_PICKUP)),
                                        new YieldCommand(200),
                                        new OneTimeCommand(() -> robot.theOuttake.setCurrentOuttakeState(Outtake.OuttakeServoState.SPECIMEN_PICKUP)),
                                        new OneTimeCommand(() -> robot.theOuttake.setCurrentPivotState(Outtake.OuttakePivotStates.SPECIMEN_PICKUP)),
                                        new OneTimeCommand(() -> robot.theOuttake.setCurrentClawState(Outtake.OuttakeClawStates.SPECIMEN_PICKUP)),
                                        new OneTimeCommand(() -> robot.theOuttake.setCurrentRotationState(Outtake.OuttakeRotationStates.ROTATED)),
                                        new YieldCommand(1000, robot.theOuttake::atTargetPosition),
                                        new YieldCommand(100),
                                        new OneTimeCommand(() -> robot.theOuttake.setSlidesState(Outtake.OuttakeSlidesStates.SPECIMEN_PICKUP))
                                )
                        ),
                        new MovementCommand(
                                SpecLoadInitial,
                                SpecLoadInitial2,
                                new MovementConstants(0.2)
                        ),
                        new OneTimeCommand(() -> robot.theOuttake.setCurrentClawState(Outtake.OuttakeClawStates.CLOSED)),
                        new YieldCommand(1000),
                        new MovementCommand(
                                SpecLoadInitial2,
                                SpecLoadFinal,
                                new MovementConstants(0.2)
                        ),
                        new OneTimeCommand(() -> {
                            robot.theOuttake.setSlidesState(Outtake.OuttakeSlidesStates.SPECIMENS_DROP);
                            robot.theOuttake.setCurrentClawState(Outtake.OuttakeClawStates.LESS_CLOSED);
                        }),
                        new MovementCommand(
                                SpecLoadFinal,
                                SpecLoadFinal2,
                                new MovementConstants(0.2)
                        ),
                        new OneTimeCommand(() -> robot.theOuttake.setCurrentClawState(Outtake.OuttakeClawStates.DEFAULT)),
                        new ParallelCommandGroup(
                                new MovementCommand(
                                        SpecLoadFinal2,
                                        SpecLoadInitial,
                                        new MovementConstants(0.2)
                                ),
                                new SequentialCommandGroup(
                                        new YieldCommand(200),
                                        new OneTimeCommand(() -> robot.theOuttake.setSlidesState(Outtake.OuttakeSlidesStates.SPECIMEN_INITIAL_PICKUP)),
                                        new YieldCommand(200),
                                        new OneTimeCommand(() -> robot.theOuttake.setCurrentOuttakeState(Outtake.OuttakeServoState.SPECIMEN_PICKUP)),
                                        new OneTimeCommand(() -> robot.theOuttake.setCurrentPivotState(Outtake.OuttakePivotStates.SPECIMEN_PICKUP)),
                                        new OneTimeCommand(() -> robot.theOuttake.setCurrentClawState(Outtake.OuttakeClawStates.SPECIMEN_PICKUP)),
                                        new OneTimeCommand(() -> robot.theOuttake.setCurrentRotationState(Outtake.OuttakeRotationStates.ROTATED)),
                                        new YieldCommand(1000, robot.theOuttake::atTargetPosition),
                                        new YieldCommand(100),
                                        new OneTimeCommand(() -> robot.theOuttake.setSlidesState(Outtake.OuttakeSlidesStates.SPECIMEN_PICKUP))
                                )
                        ),
                        new MovementCommand(
                                SpecLoadInitial,
                                SpecLoadInitial2,
                                new MovementConstants(0.2)
                        ),
                        new OneTimeCommand(() -> robot.theOuttake.setCurrentClawState(Outtake.OuttakeClawStates.CLOSED)),
                        new YieldCommand(1000),
                        new MovementCommand(
                                SpecLoadInitial2,
                                SpecLoadFinal,
                                new MovementConstants(0.2)
                        ),
                        new OneTimeCommand(() -> {
                            robot.theOuttake.setSlidesState(Outtake.OuttakeSlidesStates.SPECIMENS_DROP);
                            robot.theOuttake.setCurrentClawState(Outtake.OuttakeClawStates.LESS_CLOSED);
                        }),
                        new MovementCommand(
                                SpecLoadFinal,
                                SpecLoadFinal2,
                                new MovementConstants(0.2)
                        ),
                        new OneTimeCommand(() -> robot.theOuttake.setCurrentClawState(Outtake.OuttakeClawStates.DEFAULT)),
                        new ParallelCommandGroup(
                                new MovementCommand(
                                        SpecLoadFinal2,
                                        SpecLoadInitial,
                                        new MovementConstants(0.2)
                                ),
                                new SequentialCommandGroup(
                                        new YieldCommand(200),
                                        new OneTimeCommand(() -> robot.theOuttake.setSlidesState(Outtake.OuttakeSlidesStates.SPECIMEN_INITIAL_PICKUP)),
                                        new YieldCommand(200),
                                        new OneTimeCommand(() -> robot.theOuttake.setCurrentOuttakeState(Outtake.OuttakeServoState.SPECIMEN_PICKUP)),
                                        new OneTimeCommand(() -> robot.theOuttake.setCurrentPivotState(Outtake.OuttakePivotStates.SPECIMEN_PICKUP)),
                                        new OneTimeCommand(() -> robot.theOuttake.setCurrentClawState(Outtake.OuttakeClawStates.SPECIMEN_PICKUP)),
                                        new OneTimeCommand(() -> robot.theOuttake.setCurrentRotationState(Outtake.OuttakeRotationStates.ROTATED)),
                                        new YieldCommand(1000, robot.theOuttake::atTargetPosition),
                                        new YieldCommand(100),
                                        new OneTimeCommand(() -> robot.theOuttake.setSlidesState(Outtake.OuttakeSlidesStates.SPECIMEN_PICKUP))
                                )
                        ),
                        new MovementCommand(
                                SpecLoadInitial,
                                SpecLoadInitial2,
                                new MovementConstants(0.2)
                        )*/

                )
        );


        SequentialCommandGroup retryCommand = new RetryCommand(robot, 0);

        robot.theOuttake.setCurrentClawState(Outtake.OuttakeClawStates.CLOSED);
        robot.theIntake.leftServo.setPosition(Intake.LinkageStates.DEFAULT.position - 0.03);
        robot.theIntake.rightServo.setPosition(Intake.LinkageStates.DEFAULT.position - 0.03);


        Gamepad gamepad1Copy = new Gamepad();
        Gamepad gamepad2Copy = new Gamepad();

        double offset = 0;

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

            telemetry.addLine("Press cross (gamepad1) to close claw");
            telemetry.addLine("Press circle (gamepad1) to set intake dropdown servos to AUTO_DEFAULT");
            telemetry.addLine("Press triangle (gamepad1) to set alliance to RED");
            telemetry.addLine("Press square (gamepad1) to set alliance to BLUE");

            telemetry.addData("Alliance: ", Globals.ALLIANCE);

            telemetry.update();
            gamepad1.copy(gamepad1Copy);
        }
        // Notify subsystems before loop
        robot.postStart();
        robot.theIntake.setAutomation(false);

        if (isStopRequested()) return;

        // Initialize variables for loop
        Gamepad currentFrameGamepad1 = new Gamepad();
        Gamepad currentFrameGamepad2 = new Gamepad();

        Gamepad previousFrameGamepad1 = new Gamepad();
        Gamepad previousFrameGamepad2 = new Gamepad();

        // robot.drivetrain.enableAntiTip();

        robot.update();

        ElapsedTime e = new ElapsedTime();

        //  robot.localizer.setPose(new Pose(-59, 15, Math.PI/2), true);


        ElapsedTime timer = new ElapsedTime();

        PIDDrive drive = new PIDDrive(robot, this, telemetry);

        robot.theLocalizer.setPose(startPose);

        robot.update();

        robot.theCommandScheduler.scheduleCommand(preloadedSpecimen);


        while (!isStopRequested()) {


            telemetry.addData("Sample contained: ", robot.theIntake.sampleContained);

            //telemetry.addData("Done with preloads: ", doneWithPreloads);
            //telemetry.addData("Done with initial: ", doneWithInitial);
            robot.update();
        }


    }
}
//}
