package frc.robot.autonomous.routines;

import java.util.function.BiFunction;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.autonomous.AutoRoutine;
import frc.robot.autonomous.TrajectoryBuilder;
import frc.robot.commands.AutoRunShooter;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Tower;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class DriveOutStraight implements AutoRoutine{

    private Intake mIntake;
    private Shooter mShooter;
    private Conveyor mConveyor;
    private Tower mTower;
    
    public DriveOutStraight(Intake intake, Shooter shooter, Conveyor conveyor, Tower tower){
        mIntake = intake;
        mShooter = shooter;
        mConveyor = conveyor;
        mTower = tower;
    }
    
    @Override
    public String getName() {
        return "DriveOutStraight";
    }

    @Override
    public Command getCommand(TrajectoryBuilder trajectoryBuilder,
            BiFunction<Trajectory, Boolean, Command> RamseteCommandBuilder) {
        return new SequentialCommandGroup(
            RamseteCommandBuilder.apply(trajectoryBuilder.getTrajectory("Blue-S3-B3"), true),
            new InstantCommand(() -> Timer.delay(2)),
            new AutoRunShooter(mConveyor, mShooter, mTower, 0.61, 60),
            new InstantCommand(() -> {
                mConveyor.stopConveyor();
                mShooter.stopShooter();
            }),
            new InstantCommand(mTower::goHome, mTower)
        );
    }
    
}
