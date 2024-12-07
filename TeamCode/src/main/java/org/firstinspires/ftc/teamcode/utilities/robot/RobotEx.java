package org.firstinspires.ftc.teamcode.utilities.robot;

import android.annotation.SuppressLint;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.lynx.commands.core.LynxI2cConfigureChannelCommand;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.utilities.robot.command.framework.CommandScheduler;
import org.firstinspires.ftc.teamcode.utilities.robot.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.utilities.robot.subsystems.Intake;
import org.firstinspires.ftc.teamcode.utilities.robot.subsystems.Limelight;
import org.firstinspires.ftc.teamcode.utilities.robot.subsystems.OpticalOdometry;
import org.firstinspires.ftc.teamcode.utilities.robot.subsystems.Outtake;
import org.firstinspires.ftc.teamcode.utilities.robot.subsystems.Subsystem;


import java.util.List;

/*
Robot Configuration
Control Hub
    Motors:
        0 - rightBackMotor
        1 - rightFrontMotor
        2 - leftFrontMotor
        3 - leftBackMotor
    Servos:
        0 - intakeClaw
        1 - boxOpen
        2 - rightBox
        3 - leftBox
        4 - leftIntakeServo
        5 - rightIntakeServo
    DPIO
        0/1 - breakBeams
        2/3 - leftProximity
        4/5 - centerProximity
        6/7 - rightProximity
Expansion Hub
    Motors:
    Servos:

 */
public class RobotEx {
    private static RobotEx theRobotInstance = null;

    List<LynxModule> theHubs;

    public CommandScheduler theCommandScheduler = CommandScheduler.getInstance();

    public Drivetrain theDrivetrain = new Drivetrain();
    public OpticalOdometry theOpticalOdometry = new OpticalOdometry();
    public Outtake theOuttake = new Outtake();
    public Intake theIntake = new Intake();
    public Limelight theLimelight = new Limelight();

    public VoltageSensor theVoltageSensor;

    private final ElapsedTime theFrameTimer = new ElapsedTime();

    private final Subsystem[] theSubsystems = new Subsystem[]{
            theDrivetrain,
            theOpticalOdometry,
            theOuttake,
            theIntake,
            theLimelight
    };

    Telemetry theTelemetry;

    public HardwareMap theHardwareMap;

    public LinearOpMode theOpMode;

    public boolean stopRequested = false;
    public double runTime = 0;

    private double voltageCompensator = 12;
    private double frames = 0;
    private double currentFrames = 0;
    private double lastTime = 0;


    private RobotEx() {
        if (RobotEx.theRobotInstance != null) {
            throw new IllegalStateException("Robot already instantiated");
        }
    }

    public static RobotEx getInstance() {
        if (RobotEx.theRobotInstance == null) {
            RobotEx.theRobotInstance = new RobotEx();
        }

        return RobotEx.theRobotInstance;
    }



    public void init(LinearOpMode opMode, Telemetry telemetry) {
        this.theOpMode = opMode;
        this.theHardwareMap = opMode.hardwareMap;
        this.theTelemetry = telemetry;

        this.theVoltageSensor = theHardwareMap.voltageSensor.iterator().next();
        this.voltageCompensator = this.theVoltageSensor.getVoltage();

        for (Subsystem subsystem : this.theSubsystems) {
            subsystem.onInit(theHardwareMap, telemetry);
        }

        telemetry.update();
    }

    public void init(LinearOpMode opMode) {
        this.init(opMode, opMode.telemetry);
    }

    public void postStart() {

        stopRequested = theOpMode.isStopRequested();

        if (stopRequested) return;

        this.theHubs = theHardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : theHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
            LynxI2cConfigureChannelCommand command = new LynxI2cConfigureChannelCommand(hub, 1, LynxI2cConfigureChannelCommand.SpeedCode.FAST_400K);

            try {
                command.send();
            } catch (Exception e) {
                // Fail
            }

        }


        for (Subsystem subsystem : this.theSubsystems) {
            subsystem.onOpmodeStarted();
        }

    }

    @SuppressLint("")
    public double update() {

        stopRequested = theOpMode.isStopRequested();

        if (stopRequested) return 0;

        runTime = theOpMode.getRuntime();

        if (Math.floor(runTime) != lastTime) {
            frames = currentFrames;
            currentFrames = 0;
            lastTime = Math.floor(runTime);
        }

        currentFrames += 1;

        ElapsedTime log = new ElapsedTime();

        log.reset();

        double startTime = theFrameTimer.milliseconds();

        for (LynxModule hub : theHubs) {
            hub.clearBulkCache();
        }

        theTelemetry.addData("End Cache Clear time: ", theFrameTimer.milliseconds() - startTime);

        theCommandScheduler.update();

        for (Subsystem subsystem : theSubsystems) {
            subsystem.onCyclePassed();
            theTelemetry.addData("Subsystem Update: ", theFrameTimer.milliseconds() - startTime);
        }

        theTelemetry.addLine("Refresh Rate: " + frames + " hz");
        theTelemetry.addData("Run time: ", runTime);

        theTelemetry.update();

        double frameTime = theFrameTimer.milliseconds();
        theFrameTimer.reset();

        return frameTime;
    }

    public void pause(double seconds) {
        ElapsedTime elapsedTime = new ElapsedTime();
        elapsedTime.reset();

        while (elapsedTime.seconds() < seconds && !stopRequested) {
            update();
        }
    }

    public void persistData() {
        PersistentData.startPose = this.theOpticalOdometry.getPose();
    }

    public double getVoltage() {
        return this.theVoltageSensor.getVoltage();
    }

    public double getPowerMultiple() {
        return 12 / this.voltageCompensator;
    }

    public void destroy() {
        RobotEx.theRobotInstance = null;
    }



}