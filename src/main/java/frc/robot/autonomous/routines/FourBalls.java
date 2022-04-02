package frc.robot.autonomous.routines;

import java.util.function.BiFunction;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.autonomous.AutoRoutine;
import frc.robot.autonomous.TrajectoryBuilder;
import frc.robot.commands.AngleTowerSetpoint;
import frc.robot.commands.DeployIntake;
import frc.robot.commands.RunIntakeAndConveyor;
import frc.robot.commands.RunShooterPID;
import frc.robot.commands.RunShooterPIDToSetpoint;
import frc.robot.commands.ShootBallSequence;
import frc.robot.commands.HomeTowerAndRetractIntake;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class FourBalls implements AutoRoutine{

    private static final double WAIT_TIME_SECONDS = 1.5;
    private static final double SHOOTER_SPEED = 50;

    private Intake mIntake = Intake.getInstance();
    private Shooter mShooter = Shooter.getInstance();
    
    @Override
    public String getName() {
        return "Four Balls";
    }

    @Override
    public Command getCommand(TrajectoryBuilder trajectoryBuilder,
            BiFunction<Trajectory, Boolean, Command> RamseteCommandBuilder) {
        return new SequentialCommandGroup(
            new ParallelCommandGroup(
                RamseteCommandBuilder.apply(trajectoryBuilder.getTrajectory("Blue-S2-B2"), true),                
                new SequentialCommandGroup(
                    new DeployIntake(),
                    new RunIntakeAndConveyor(),
                    new AngleTowerSetpoint(60)
                )
            ),
            new WaitCommand(0.5),
            // new InstantCommand(() -> {
            //     mIntake.stopRoller();
            // }, mIntake),
            new ShootBallSequence(SHOOTER_SPEED),
            new InstantCommand(() -> {
                mShooter.stopShooter();
            }, mShooter),
            new ParallelCommandGroup(
                RamseteCommandBuilder.apply(trajectoryBuilder.getTrajectory("Blue-B2-T"), false),
                new RunIntakeAndConveyor()
            ),
            new WaitCommand(WAIT_TIME_SECONDS),
            RamseteCommandBuilder.apply(trajectoryBuilder.getTrajectory("Blue-T-Shoot"), false),
            new InstantCommand(mIntake::stopRoller, mIntake),
            new ShootBallSequence(SHOOTER_SPEED),
            new InstantCommand(()->{
                mShooter.stopShooter();
            }, mShooter),
            new HomeTowerAndRetractIntake() 
        );
    }
    
}
