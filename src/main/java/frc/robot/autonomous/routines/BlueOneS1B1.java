package frc.robot.autonomous.routines;

import java.util.function.BiFunction;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.autonomous.AutoRoutine;
import frc.robot.autonomous.TrajectoryBuilder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class BlueOneS1B1 implements AutoRoutine{

    private Intake mIntake;
    private Shooter mShooter;
    
    public BlueOneS1B1(Intake intake, Shooter shooter){
        mIntake = intake;
        mShooter = shooter;
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
                new InstantCommand(()->mIntake.deploy(), mIntake)
            ),
            new RunCommand(()->mIntake.runRoller(Constants.INTAKE_ROLLER_SPEED), mIntake),
            new RunCommand(()->mShooter.runShooter(0.5), mShooter)
        );
    }
    
}
