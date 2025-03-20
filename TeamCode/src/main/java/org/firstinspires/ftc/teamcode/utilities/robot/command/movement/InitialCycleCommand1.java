package org.firstinspires.ftc.teamcode.utilities.robot.command.movement;

import static org.firstinspires.ftc.teamcode.opmodes.auto.SampleCycleAuto.cycleInitial;
import static org.firstinspires.ftc.teamcode.opmodes.auto.SampleCycleAuto.cycleStrafe;
import static org.firstinspires.ftc.teamcode.opmodes.auto.SampleCycleAuto.cycleSubmersible;
import static org.firstinspires.ftc.teamcode.opmodes.auto.SampleCycleAuto.cycleSubmersible2;
import static org.firstinspires.ftc.teamcode.opmodes.auto.SampleCycleAuto.cycleSubmersible3;
import static org.firstinspires.ftc.teamcode.opmodes.auto.SampleCycleAuto.sample3Drop;

import org.firstinspires.ftc.teamcode.utilities.math.linearalgebra.Pose;
import org.firstinspires.ftc.teamcode.utilities.robot.DriveConstants;
import org.firstinspires.ftc.teamcode.utilities.robot.RobotEx;
import org.firstinspires.ftc.teamcode.utilities.robot.command.framework.commandtypes.DeadlineCommand;
import org.firstinspires.ftc.teamcode.utilities.robot.command.framework.commandtypes.OneTimeCommand;
import org.firstinspires.ftc.teamcode.utilities.robot.command.framework.commandtypes.ParallelCommandGroup;
import org.firstinspires.ftc.teamcode.utilities.robot.command.framework.commandtypes.SequentialCommandGroup;
import org.firstinspires.ftc.teamcode.utilities.robot.command.framework.commandtypes.YieldCommand;
import org.firstinspires.ftc.teamcode.utilities.robot.movement.MovementConstants;
import org.firstinspires.ftc.teamcode.utilities.robot.subsystems.Intake;

public class InitialCycleCommand1 extends SequentialCommandGroup {
    public InitialCycleCommand1(RobotEx robot, double offset) {
        super(
                new ParallelCommandGroup(
                        new MovementCommand(
                                sample3Drop,
                                cycleInitial,
                                new MovementConstants(80, 80, -0.5, DriveConstants.K_V, DriveConstants.K_A)
                        ),
                        new SequentialCommandGroup(
                                new YieldCommand(100),
                                new OneTimeCommand(() -> robot.theOuttake.reset())
                        )
                ),
                new OneTimeCommand(() -> robot.theIntake.setTargetLinkageState(Intake.LinkageStates.AUTO_EXTENSION)),
                new MovementCommand(
                        cycleInitial,
                        new Pose(cycleSubmersible.getX(), cycleSubmersible.getY() + offset, cycleSubmersible.getHeading()),
                        new MovementConstants(40, 40, -0.2, DriveConstants.K_V, DriveConstants.K_A)
                ),
                new OneTimeCommand(() -> robot.theIntake.triggerCowcatcher()),
                new YieldCommand(0),
                new OneTimeCommand(() -> robot.theIntake.setIntakeState(Intake.IntakeState.EXTENDED)),
                new OneTimeCommand(() -> robot.theIntake.setIntakeMotorState(Intake.IntakeMotorStates.INTAKING)),
                new YieldCommand(50),
                new OneTimeCommand(() -> robot.theIntake.setTargetLinkageState(Intake.LinkageStates.EXTENDED)),
                new DeadlineCommand(
                        new YieldCommand(robot.theIntake::containsSampleColorSensor),
                        new SequentialCommandGroup(
                                new MovementCommand(
                                        new Pose(cycleSubmersible.getX(), cycleSubmersible.getY() + offset, cycleSubmersible.getHeading()),
                                        new Pose(cycleStrafe.getX(), cycleStrafe.getY() + offset, cycleStrafe.getHeading()),
                                        new MovementConstants(0.2)
                                ),
                                new MovementCommand(
                                        new Pose(cycleStrafe.getX(), cycleStrafe.getY() + offset, cycleStrafe.getHeading()),
                                        new Pose(cycleSubmersible2.getX(), cycleSubmersible2.getY() + offset, cycleSubmersible2.getHeading()),
                                        new MovementConstants(-0.25)
                                ),
                                new MovementCommand(
                                        new Pose(cycleSubmersible2.getX(), cycleSubmersible2.getY() + offset, cycleSubmersible2.getHeading()),
                                        new Pose(cycleSubmersible3.getX(), cycleSubmersible3.getY() + offset, cycleSubmersible3.getHeading()),
                                        new MovementConstants(0.5)
                                )
                        )
                ),
                new YieldCommand(100)
        );
    }
}