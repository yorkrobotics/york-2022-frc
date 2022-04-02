package frc.robot.autonomous.routines;

import java.util.function.BiFunction;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.autonomous.AutoRoutine;
import frc.robot.autonomous.TrajectoryBuilder;
import frc.robot.commands.ReverseDriveTimed;
import frc.robot.commands.AngleTowerSetpoint;
import frc.robot.commands.DeployIntake;
import frc.robot.commands.RetractIntake;
import frc.robot.commands.RunIntakeAndConveyor;
import frc.robot.commands.ShootBallSequence;
import frc.robot.commands.HomeTower;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class DriveOutStraight implements AutoRoutine{

    private Intake mIntake = Intake.getInstance();
    private Shooter mShooter = Shooter.getInstance();
    private Conveyor mConveyor = Conveyor.getInstance();
    
    @Override
    public String getName() {
        return "DriveOutStraight";
    }

    @Override
    public Command getCommand(TrajectoryBuilder trajectoryBuilder,
            BiFunction<Trajectory, Boolean, Command> RamseteCommandBuilder) {
        return new SequentialCommandGroup(
            new ParallelCommandGroup(
                new ReverseDriveTimed(1),
                new SequentialCommandGroup(
                    new DeployIntake(),
                    new RunIntakeAndConveyor(),
                    new AngleTowerSetpoint(60)
                )
            ),
            new WaitCommand(2),
            new InstantCommand(() -> {
                mIntake.stopRoller();
                mConveyor.stopConveyor();
            }, mIntake, mConveyor),
            new ShootBallSequence(48),
            new InstantCommand(() -> {
                mConveyor.stopConveyor();
                mShooter.stopShooter();
            }),
            new HomeTower(),
            new RetractIntake()
        );
    }
    
}
