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

public class ApproachCommand extends SequentialCommandGroup {
    public ApproachCommand(RobotEx robot, double offset) {
        super(
                new ParallelCommandGroup(
                        new MovementCommand(
                                sample3Drop,
                                cycleInitial,
                                new MovementConstants(80, 80, -0.5, DriveConstants.K_V, DriveConstants.K_A)
                        ),
                        new SequentialCommandGroup(
                                new YieldCommand(100),
                                new OneTimeCommand(() -> robot.theOuttake.reset()),
                                new YieldCommand(500),
                                new OneTimeCommand(() -> robot.theIntake.setTargetLinkageState(Intake.LinkageStates.EXTENDED))
                        )
                ),
                new MovementCommand(
                        cycleInitial,
                        new Pose(cycleSubmersible.getX(), cycleSubmersible.getY() + offset, cycleSubmersible.getHeading()),
                        new MovementConstants(40, 40, -0.2, DriveConstants.K_V, DriveConstants.K_A)
                ),
                new OneTimeCommand(() -> robot.theIntake.triggerCowcatcher())

        );
    }
}
