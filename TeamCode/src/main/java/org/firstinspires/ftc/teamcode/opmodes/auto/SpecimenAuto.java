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

    public static Pose startPose = new Pose(1.5, 0.5,  0);
    public static Pose Spec1 = new Pose(32.4, 4.74, -0.02);
    public static Pose Spec2Pickup = new Pose(22.5, -20.6, -0.90);
    public static Pose Spec2Spit = new Pose(19.18, -24.55, -2.3);
    public static Pose Spec3Pickup = new Pose(24, -28, -1);
    public static Pose Spec3Spit = new Pose(19.21, -28.13, -2.29);
    public static Pose Spec4Pickup = new Pose(24.57, -35.71, -1);
    public static Pose Spec4Spit = new Pose(8.58, -1.95, -1.95);
    public static Pose SpecLoadInitial = new Pose(5, -25.87, 0);
    public static Pose SpecLoadFinal = new Pose(-0.5, -25.87, 0);
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
                        new MovementConstants(0.2)
                ),
                new OneTimeCommand(() -> robot.theOuttake.setSlidesState(Outtake.OuttakeSlidesStates.SPECIMENS_DROP)),
                new OneTimeCommand(() -> robot.theOuttake.setCurrentClawState(Outtake.OuttakeClawStates.LESS_CLOSED)),
                new YieldCommand(200),//wait for slides to move down


                new ParallelCommandGroup(
                        new MovementCommand(
                                Spec1,
                                Spec2Pickup,
                                new MovementConstants(0.2)
                        ),
                        new SequentialCommandGroup(
                            new OneTimeCommand(() -> robot.theOuttake.setSlidesState(Outtake.OuttakeSlidesStates.SPECIMEN_INITIAL_PICKUP)),
                            new YieldCommand(200),
                            new OneTimeCommand(() -> robot.theOuttake.setCurrentOuttakeState(Outtake.OuttakeServoState.SPECIMEN_PICKUP)),
                            new OneTimeCommand(() -> robot.theOuttake.setCurrentPivotState(Outtake.OuttakePivotStates.SPECIMEN_PICKUP)),
                            new OneTimeCommand(() -> robot.theOuttake.setCurrentClawState(Outtake.OuttakeClawStates.SPECIMEN_PICKUP)),
                            new OneTimeCommand(() -> robot.theOuttake.setCurrentRotationState(Outtake.OuttakeRotationStates.ROTATED)),
                            new YieldCommand(1000, robot.theOuttake::atTargetPosition),
                            new YieldCommand(400),
                            new OneTimeCommand(() -> robot.theOuttake.setSlidesState(Outtake.OuttakeSlidesStates.SPECIMEN_PICKUP))
                        )//spec pickup ready
                ),
                new SequentialCommandGroup(
                new OneTimeCommand(() -> robot.theIntake.setTargetLinkageState(Intake.LinkageStates.EXTENDED)),
                new OneTimeCommand(() -> robot.theIntake.setIntakeState(Intake.IntakeState.EXTENDED)),
                new OneTimeCommand(() -> robot.theIntake.setIntakeMotorState(Intake.IntakeMotorStates.INTAKING)),
                        new YieldCommand(600)//pickup
                ),
                new ParallelCommandGroup(
                        new OneTimeCommand(() -> robot.theIntake.setIntakeState(Intake.IntakeState.EJECT)),
                        new OneTimeCommand(() -> robot.theIntake.setTargetLinkageState(Intake.LinkageStates.AUTO_ROTATE)),
                    new MovementCommand(
                        Spec2Pickup,
                        Spec2Spit,
                        new MovementConstants(0)
                    )
                ),
                new OneTimeCommand(() -> robot.theIntake.setIntakeMotorState(Intake.IntakeMotorStates.HARD_REVERSE)),
                    new YieldCommand(300),//spit out
                new OneTimeCommand(() -> robot.theIntake.setIntakeMotorState(Intake.IntakeMotorStates.INTAKING)),
                new MovementCommand(
                        Spec2Spit,
                        Spec3Pickup,
                        new MovementConstants(0.2)
                ),
                new SequentialCommandGroup(
                        new OneTimeCommand(() -> robot.theIntake.setIntakeState(Intake.IntakeState.EXTENDED)),
                    new OneTimeCommand(() -> robot.theIntake.setTargetLinkageState(Intake.LinkageStates.EXTENDED)),
                    new YieldCommand(600)//suk
                ),
                new ParallelCommandGroup(
                        new OneTimeCommand(() -> robot.theIntake.setIntakeState(Intake.IntakeState.EJECT)),
                        new OneTimeCommand(() -> robot.theIntake.setTargetLinkageState(Intake.LinkageStates.AUTO_ROTATE)),
                    new MovementCommand(
                        Spec3Pickup,
                        Spec3Spit,
                        new MovementConstants(0)
                    )
                ),
                new OneTimeCommand(() -> robot.theIntake.setIntakeMotorState(Intake.IntakeMotorStates.HARD_REVERSE)),
                new YieldCommand(300),//spit out
                new OneTimeCommand(() -> robot.theIntake.setIntakeMotorState(Intake.IntakeMotorStates.INTAKING)),
                new MovementCommand(
                        Spec3Spit,
                        Spec4Pickup,
                        new MovementConstants(0.4)
                ),
                        new SequentialCommandGroup(
                                new OneTimeCommand(() -> robot.theIntake.setIntakeState(Intake.IntakeState.EXTENDED)),
                                new OneTimeCommand(() -> robot.theIntake.setTargetLinkageState(Intake.LinkageStates.EXTENDED)),
                                new YieldCommand(600)//suk
                        ),
                        new ParallelCommandGroup(
                                new OneTimeCommand(() -> robot.theIntake.setIntakeState(Intake.IntakeState.EJECT)),
                    new OneTimeCommand(() -> robot.theIntake.setTargetLinkageState(Intake.LinkageStates.AUTO_ROTATE)),
                    new MovementCommand(
                        Spec4Pickup,
                        Spec4Spit,
                        new MovementConstants(0)
                    )
                ),
                 new OneTimeCommand(() -> robot.theIntake.setIntakeMotorState(Intake.IntakeMotorStates.HARD_REVERSE)),
                 new YieldCommand(300),//spit out
                new ParallelCommandGroup( 
                    new OneTimeCommand(() -> robot.theIntake.returnSlides()),
                    new MovementCommand(
                        Spec4Spit,
                        SpecLoadInitial,
                        new MovementConstants(0.5)
                )
                        ),
                   new SequentialCommandGroup(
                           new YieldCommand(500),
                       new ParallelCommandGroup(
                       new OneTimeCommand(() -> robot.theOuttake.setCurrentClawState(Outtake.OuttakeClawStates.SPECIMEN_PICKUP)),
                       new MovementCommand(
                        SpecLoadInitial,
                        SpecLoadFinal,
                        new MovementConstants(0.5)
                          )
                        ),
                        new OneTimeCommand(() -> robot.theOuttake.setCurrentClawState(Outtake.OuttakeClawStates.CLOSED)),
                        new YieldCommand(200),
                           new ParallelCommandGroup(
                                   new MovementCommand(
                                           SpecLoadFinal,
                                           Spec2PlaceInitial,
                                           new MovementConstants(0.5)
                                   ),
                                   new SequentialCommandGroup(
                                   new OneTimeCommand(() -> robot.theIntake.leftDropdownServo.setPosition(Intake.IntakeState.AUTO_DEFAULT.position)),
                                   new OneTimeCommand(() -> robot.theIntake.rightDropdownServo.setPosition(Intake.IntakeState.AUTO_DEFAULT.position)),
                                   new OneTimeCommand(() -> robot.theOuttake.setCurrentClawState(Outtake.OuttakeClawStates.CLOSED)),
                                   new OneTimeCommand(() -> robot.theOuttake.setSlidesState(Outtake.OuttakeSlidesStates.SPECIMENS)),
                                   new OneTimeCommand(() -> robot.theOuttake.setCurrentOuttakeState(Outtake.OuttakeServoState.SPECIMEN_INITIAL)),
                                   new OneTimeCommand(() -> robot.theOuttake.setCurrentPivotState(Outtake.OuttakePivotStates.SPECIMEN_DROP)),
                                   new OneTimeCommand(() -> robot.theOuttake.setCurrentRotationState(Outtake.OuttakeRotationStates.SPECIMEN_ROTATED))
                                   )
                           ),
                       new MovementCommand(
                        Spec2PlaceInitial,
                        Spec2PlaceFinal,
                        new MovementConstants(0)
                       ),
                new OneTimeCommand(() -> robot.theOuttake.setSlidesState(Outtake.OuttakeSlidesStates.SPECIMENS_DROP)),
                new OneTimeCommand(() -> robot.theOuttake.setCurrentClawState(Outtake.OuttakeClawStates.LESS_CLOSED)),
                new YieldCommand(300),//wait for slides to move down
        new ParallelCommandGroup(
                new SequentialCommandGroup(
                new YieldCommand(400),      //wait to move back slighltly
                new OneTimeCommand(() -> robot.theOuttake.setSlidesState(Outtake.OuttakeSlidesStates.SPECIMEN_INITIAL_PICKUP)),
                new YieldCommand(200),
                new OneTimeCommand(() -> robot.theOuttake.setCurrentOuttakeState(Outtake.OuttakeServoState.SPECIMEN_PICKUP)),
                new OneTimeCommand(() -> robot.theOuttake.setCurrentPivotState(Outtake.OuttakePivotStates.SPECIMEN_PICKUP)),
                new OneTimeCommand(() -> robot.theOuttake.setCurrentClawState(Outtake.OuttakeClawStates.SPECIMEN_PICKUP)),
                new OneTimeCommand(() -> robot.theOuttake.setCurrentRotationState(Outtake.OuttakeRotationStates.ROTATED)),
                new YieldCommand(1000, robot.theOuttake::atTargetPosition),
                new YieldCommand(400),
                new OneTimeCommand(() -> robot.theOuttake.setSlidesState(Outtake.OuttakeSlidesStates.SPECIMEN_PICKUP))
                ),
                new MovementCommand(
                        Spec2PlaceFinal,
                        SpecLoadInitial,
                        new MovementConstants(0.2)
                ),
            new MovementCommand(
                SpecLoadInitial,
                SpecLoadFinal,
                new MovementConstants(0)
        )
        ),
                new OneTimeCommand(() -> robot.theOuttake.setCurrentClawState(Outtake.OuttakeClawStates.CLOSED)),
                new YieldCommand(200),
                new ParallelCommandGroup(
                        new MovementCommand(
                                SpecLoadFinal,
                                Spec3PlaceInitial,
                                new MovementConstants(0.2)
                        ),
                        new SequentialCommandGroup(
                        new OneTimeCommand(() -> robot.theIntake.leftDropdownServo.setPosition(Intake.IntakeState.AUTO_DEFAULT.position)),
                        new OneTimeCommand(() -> robot.theIntake.rightDropdownServo.setPosition(Intake.IntakeState.AUTO_DEFAULT.position)),
                        new OneTimeCommand(() -> robot.theOuttake.setCurrentClawState(Outtake.OuttakeClawStates.CLOSED)),
                        new OneTimeCommand(() -> robot.theOuttake.setSlidesState(Outtake.OuttakeSlidesStates.SPECIMENS)),
                        new OneTimeCommand(() -> robot.theOuttake.setCurrentOuttakeState(Outtake.OuttakeServoState.SPECIMEN_INITIAL)),
                        new OneTimeCommand(() -> robot.theOuttake.setCurrentPivotState(Outtake.OuttakePivotStates.SPECIMEN_DROP)),
                        new OneTimeCommand(() -> robot.theOuttake.setCurrentRotationState(Outtake.OuttakeRotationStates.SPECIMEN_ROTATED))
                        )
                ),
                new MovementCommand(
                        Spec3PlaceInitial,
                        Spec3PlaceFinal,
                        new MovementConstants(0)
                ),
                new OneTimeCommand(() -> robot.theOuttake.setSlidesState(Outtake.OuttakeSlidesStates.SPECIMENS_DROP)),
                new OneTimeCommand(() -> robot.theOuttake.setCurrentClawState(Outtake.OuttakeClawStates.LESS_CLOSED)),
                new YieldCommand(300),//wait for slides to move down
        new ParallelCommandGroup(
                new SequentialCommandGroup(
                        new YieldCommand(400),      //wait to move back slighltly
                        new OneTimeCommand(() -> robot.theOuttake.setSlidesState(Outtake.OuttakeSlidesStates.SPECIMEN_INITIAL_PICKUP)),
                        new YieldCommand(200),
                        new OneTimeCommand(() -> robot.theOuttake.setCurrentOuttakeState(Outtake.OuttakeServoState.SPECIMEN_PICKUP)),
                        new OneTimeCommand(() -> robot.theOuttake.setCurrentPivotState(Outtake.OuttakePivotStates.SPECIMEN_PICKUP)),
                        new OneTimeCommand(() -> robot.theOuttake.setCurrentClawState(Outtake.OuttakeClawStates.SPECIMEN_PICKUP)),
                        new OneTimeCommand(() -> robot.theOuttake.setCurrentRotationState(Outtake.OuttakeRotationStates.ROTATED)),
                        new YieldCommand(1000, robot.theOuttake::atTargetPosition),
                        new YieldCommand(400),
                        new OneTimeCommand(() -> robot.theOuttake.setSlidesState(Outtake.OuttakeSlidesStates.SPECIMEN_PICKUP))
                ),
                new MovementCommand(
                        Spec3PlaceFinal,
                        SpecLoadInitial,
                        new MovementConstants(0.2)
                ),
                new MovementCommand(
                        SpecLoadInitial,
                        SpecLoadFinal,
                        new MovementConstants(0)
                )
        ),
                new OneTimeCommand(() -> robot.theOuttake.setCurrentClawState(Outtake.OuttakeClawStates.CLOSED)),
                new YieldCommand(200),
                new ParallelCommandGroup(
                        new MovementCommand(
                                SpecLoadFinal,
                                Spec4PlaceInitial,
                                new MovementConstants(0.2)
                        ),
                        new SequentialCommandGroup(
                        new OneTimeCommand(() -> robot.theIntake.leftDropdownServo.setPosition(Intake.IntakeState.AUTO_DEFAULT.position)),
                        new OneTimeCommand(() -> robot.theIntake.rightDropdownServo.setPosition(Intake.IntakeState.AUTO_DEFAULT.position)),
                        new OneTimeCommand(() -> robot.theOuttake.setCurrentClawState(Outtake.OuttakeClawStates.CLOSED)),
                        new OneTimeCommand(() -> robot.theOuttake.setSlidesState(Outtake.OuttakeSlidesStates.SPECIMENS)),
                        new OneTimeCommand(() -> robot.theOuttake.setCurrentOuttakeState(Outtake.OuttakeServoState.SPECIMEN_INITIAL)),
                        new OneTimeCommand(() -> robot.theOuttake.setCurrentPivotState(Outtake.OuttakePivotStates.SPECIMEN_DROP)),
                        new OneTimeCommand(() -> robot.theOuttake.setCurrentRotationState(Outtake.OuttakeRotationStates.SPECIMEN_ROTATED))
                        )
                ),
                new MovementCommand(
                        Spec4PlaceInitial,
                        Spec4PlaceFinal,
                        new MovementConstants(0)
                ),
                new OneTimeCommand(() -> robot.theOuttake.setSlidesState(Outtake.OuttakeSlidesStates.SPECIMENS_DROP)),
                new OneTimeCommand(() -> robot.theOuttake.setCurrentClawState(Outtake.OuttakeClawStates.LESS_CLOSED)),
                new YieldCommand(300),//wait for slides to move down
        new ParallelCommandGroup(
                new SequentialCommandGroup(
                        new YieldCommand(400),      //wait to move back slighltly
                        new OneTimeCommand(() -> robot.theOuttake.setSlidesState(Outtake.OuttakeSlidesStates.SPECIMEN_INITIAL_PICKUP)),
                        new YieldCommand(200),
                        new OneTimeCommand(() -> robot.theOuttake.setCurrentOuttakeState(Outtake.OuttakeServoState.SPECIMEN_PICKUP)),
                        new OneTimeCommand(() -> robot.theOuttake.setCurrentPivotState(Outtake.OuttakePivotStates.SPECIMEN_PICKUP)),
                        new OneTimeCommand(() -> robot.theOuttake.setCurrentClawState(Outtake.OuttakeClawStates.SPECIMEN_PICKUP)),
                        new OneTimeCommand(() -> robot.theOuttake.setCurrentRotationState(Outtake.OuttakeRotationStates.ROTATED)),
                        new YieldCommand(1000, robot.theOuttake::atTargetPosition),
                        new YieldCommand(400),
                        new OneTimeCommand(() -> robot.theOuttake.setSlidesState(Outtake.OuttakeSlidesStates.SPECIMEN_PICKUP))
                ),
                new MovementCommand(
                        Spec4PlaceFinal,
                        SpecLoadInitial,
                        new MovementConstants(0.2)
                ),
                new MovementCommand(
                        SpecLoadInitial,
                        SpecLoadFinal,
                        new MovementConstants(0)
                )
        ),
                new OneTimeCommand(() -> robot.theOuttake.setCurrentClawState(Outtake.OuttakeClawStates.CLOSED)),
                new YieldCommand(200),
                new ParallelCommandGroup(
                        new MovementCommand(
                                SpecLoadFinal,
                                Spec5PlaceInitial,
                                new MovementConstants(0.2)
                        ),
                        new SequentialCommandGroup(
                                new OneTimeCommand(() -> robot.theIntake.leftDropdownServo.setPosition(Intake.IntakeState.AUTO_DEFAULT.position)),
                                new OneTimeCommand(() -> robot.theIntake.rightDropdownServo.setPosition(Intake.IntakeState.AUTO_DEFAULT.position)),
                                new OneTimeCommand(() -> robot.theOuttake.setCurrentClawState(Outtake.OuttakeClawStates.CLOSED)),
                                new OneTimeCommand(() -> robot.theOuttake.setSlidesState(Outtake.OuttakeSlidesStates.SPECIMENS)),
                                new OneTimeCommand(() -> robot.theOuttake.setCurrentOuttakeState(Outtake.OuttakeServoState.SPECIMEN_INITIAL)),
                                new OneTimeCommand(() -> robot.theOuttake.setCurrentPivotState(Outtake.OuttakePivotStates.SPECIMEN_DROP)),
                                new OneTimeCommand(() -> robot.theOuttake.setCurrentRotationState(Outtake.OuttakeRotationStates.SPECIMEN_ROTATED))
                        )
                ),
                new MovementCommand(
                        Spec5PlaceInitial,
                        Spec5PlaceFinal,
                        new MovementConstants(0)
                ),
                new OneTimeCommand(() -> robot.theOuttake.setSlidesState(Outtake.OuttakeSlidesStates.SPECIMENS_DROP)),
                new OneTimeCommand(() -> robot.theOuttake.setCurrentClawState(Outtake.OuttakeClawStates.LESS_CLOSED)),
                new YieldCommand(300)//wait for slides to move down
                )

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






            // Notify subsystems before loop
        waitForStart();
        robot.theOuttake.setCurrentClawState(Outtake.OuttakeClawStates.CLOSED);
        robot.theOuttake.setCurrentOuttakeState(Outtake.OuttakeServoState.AUTO_DEFAULT);










                }
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

       //  robot.localizer.setPose(new Pose(-59, 15, Math.PI/2), true);


        ElapsedTime timer = new ElapsedTime();

        PIDDrive drive = new PIDDrive(robot, this, telemetry);

        robot.theLocalizer.setPose(new Pose(0, 0,  0));

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
