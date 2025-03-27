package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.utilities.math.linearalgebra.Pose;
import org.firstinspires.ftc.teamcode.utilities.robot.RobotEx;
import org.firstinspires.ftc.teamcode.utilities.robot.command.framework.commandtypes.OneTimeCommand;
import org.firstinspires.ftc.teamcode.utilities.robot.command.framework.commandtypes.SequentialCommandGroup;
import org.firstinspires.ftc.teamcode.utilities.robot.command.movement.MovementCommand;
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


      /*  SequentialCommandGroup commands = new SequentialCommandGroup(
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

*/
/**
 * Example teleop code for a basic mecanum drive
 */

@Autonomous(name = "Specimen Auto")
public class SpecimenAuto extends LinearOpMode {

    // Create new Instance of the robot
    RobotEx robot = RobotEx.getInstance();

    public static Pose startPose = new Pose(-37.1, -61.0, Math.toRadians(90));
    public static Pose Spec1 = new Pose(-59.8, -52.2, 1.1784);
    public static Pose sample1 = new Pose(-58.2995, -51.5757, 1.1784);
    public static Pose spec2Pickup = new Pose(-57, -47.3757, 1.1784);
    public static Pose spec2Spit = new Pose(-58.1997, -51.2757, 1.1784);
    public static Pose spec3Pickup = new Pose(-58.2771, -52.8033, 1.1448);
    public static Pose spec3Spit = new Pose(-52.7078, -45.9624, 2.1477);
    public static Pose spec4Pickup = new Pose(-52.7078, -45.9624, 2.1477);
    public static Pose spec4Spit = new Pose(-52.7078, -45.9624, 2.1477);
    public static Pose spec5Pickup = new Pose(-52.6078, -45.8624, 2.2477);
    public static Pose spec5Spit = new Pose(-23, -9, 0);
    public static Pose SpecLoadInitial = new Pose(-23, -3, 0);
    public static Pose SpecLoadFinal = new Pose(-23, -3, 0);
    public static Pose Spec2Place = new Pose(-27, 3, 0.5);
    public static Pose Spec3Place = new Pose(-27, 3, 0.5);
    public static Pose Spec4Place = new Pose(-27, 3, 0.5);
    public static Pose Spec5Place = new Pose(-27, 3, 0.5);
    public static Pose parkFinal = new Pose(-17, -9, Math.PI);

    public static MovementConstants defaultMovementConstants = new MovementConstants();

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.setMsTransmissionInterval(500);

        // Initialize the robot
        robot.init(this, telemetry);
        SequentialCommandGroup preloadedSpecimen = new SequentialCommandGroup(
                new OneTimeCommand(() -> robot.theOuttake.setSlidesState(Outtake.OuttakeSlidesStates.SAMPLES)),
                new MovementCommand(
                        startPose,
                        Spec1,
                        new MovementConstants(0)
                ),

        );


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
        boolean retriedPickup = false;

        while (!isStopRequested()) {



            telemetry.addData("Sample contained: ", robot.theIntake.sampleContained);
            telemetry.addData("Retry: ", retriedPickup);
            telemetry.addData("Done with preloads: ", doneWithPreloads);
            telemetry.addData("Done with initial: ", doneWithInitial);
            robot.update();
        }





    }
}
