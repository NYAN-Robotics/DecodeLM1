package org.firstinspires.ftc.teamcode.utilities.robot.command.movement;

import static org.firstinspires.ftc.teamcode.opmodes.auto.SampleCycleAuto.cycleDrop;
import static org.firstinspires.ftc.teamcode.opmodes.auto.SampleCycleAuto.cycleInitial;
import static org.firstinspires.ftc.teamcode.opmodes.auto.SampleCycleAuto.cycleSubmersible;

import org.firstinspires.ftc.teamcode.utilities.robot.DriveConstants;
import org.firstinspires.ftc.teamcode.utilities.robot.RobotEx;
import org.firstinspires.ftc.teamcode.utilities.robot.command.framework.commandtypes.OneTimeCommand;
import org.firstinspires.ftc.teamcode.utilities.robot.command.framework.commandtypes.ParallelCommandGroup;
import org.firstinspires.ftc.teamcode.utilities.robot.command.framework.commandtypes.SequentialCommandGroup;
import org.firstinspires.ftc.teamcode.utilities.robot.command.framework.commandtypes.YieldCommand;
import org.firstinspires.ftc.teamcode.utilities.robot.movement.MovementConstants;
import org.firstinspires.ftc.teamcode.utilities.robot.subsystems.Intake;
import org.firstinspires.ftc.teamcode.utilities.robot.subsystems.Outtake;

public class CycleCommand extends SequentialCommandGroup {
    public CycleCommand(RobotEx robot) {
        super(new OneTimeCommand(() -> robot.theIntake.returnSlides()),
                new OneTimeCommand(() -> robot.theIntake.setCurrentCowcatcherState(Intake.CowcatcherStates.ACTIVATED)),
                new MovementCommand(
                        cycleSubmersible,
                        cycleInitial,
                        new MovementConstants(80, 80, -0.4, DriveConstants.K_V, DriveConstants.K_A)
                ),
                new OneTimeCommand(() -> robot.theOuttake.setSlidesState(Outtake.OuttakeSlidesStates.SAMPLES)),
                new MovementCommand(
                        cycleInitial,
                        cycleDrop,
                        new MovementConstants(0.4)
                ),
                new OneTimeCommand(() -> robot.theOuttake.setCurrentClawState(Outtake.OuttakeClawStates.DEFAULT)),
                new OneTimeCommand(() -> robot.theIntake.setCurrentCowcatcherState(Intake.CowcatcherStates.DEFAULT)),
                new YieldCommand(100),
                new ParallelCommandGroup(
                        new MovementCommand(
                                cycleDrop,
                                cycleInitial,
                                new MovementConstants(80, 80, -0.2, DriveConstants.K_V, DriveConstants.K_A)
                        ),
                        new SequentialCommandGroup(
                                new YieldCommand(500),
                                new OneTimeCommand(() -> robot.theOuttake.reset()),
                                new OneTimeCommand(() -> robot.theIntake.setTargetLinkageState(Intake.LinkageStates.EXTENDED))
                        )
                ),
                new MovementCommand(
                        cycleInitial,
                        cycleSubmersible,
                        new MovementConstants(20, 40, -0.35, DriveConstants.K_V, DriveConstants.K_A)
                ),
                new OneTimeCommand(() -> robot.theIntake.setIntakeState(Intake.IntakeState.EXTENDED)),
                new OneTimeCommand(() -> robot.theIntake.setIntakeMotorState(Intake.IntakeMotorStates.INTAKING)));
    }
}
