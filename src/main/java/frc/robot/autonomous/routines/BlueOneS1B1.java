package frc.robot.autonomous.routines;

import java.util.function.BiFunction;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.autonomous.AutoRoutine;
import frc.robot.autonomous.TrajectoryBuilder;
import frc.robot.commands.RunIntakeAndConveyor;
import frc.robot.commands.AutoRunShooter;
import frc.robot.commands.DeployIntake;
import frc.robot.commands.RetractIntake;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Tower;

public class BlueOneS1B1 implements AutoRoutine{

    private Intake mIntake;
    private Shooter mShooter;
    private Conveyor mConveyor;
    private Tower mTower;
    
    public BlueOneS1B1(Intake intake, Shooter shooter, Conveyor conveyor, Tower tower){
        mIntake = intake;
        mShooter = shooter;
        mConveyor = conveyor;
        mTower = tower;
    }
    @Override
    public String getName() {
        return "Blue-OneBall-S1-B1";
    }

    @Override
    public Command getCommand(TrajectoryBuilder trajectoryBuilder,
            BiFunction<Trajectory, Boolean, Command> RamseteCommandBuilder) {
        return new SequentialCommandGroup(
            new ParallelCommandGroup(
                RamseteCommandBuilder.apply(trajectoryBuilder.getTrajectory("Blue-S1-B1"), true),
                new SequentialCommandGroup(
                    new DeployIntake(mIntake, mTower),
                    new RunIntakeAndConveyor(mIntake, mConveyor)
                )
            ),
            new InstantCommand(() -> Timer.delay(2)),
            new InstantCommand(() -> {
                mIntake.stopRoller();
                mConveyor.stopConveyor();
            }, mIntake, mConveyor),
            new AutoRunShooter(mConveyor, mShooter, mTower, 0.62, 60),
            new InstantCommand(() -> {
                mConveyor.stopConveyor();
                mShooter.stopShooter();
            }),
            new InstantCommand(mTower::goHome, mTower),
            new RetractIntake(mIntake, mTower)

        );
    }
    
}
