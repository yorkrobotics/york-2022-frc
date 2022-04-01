package frc.robot.autonomous.routines;

import java.util.function.BiFunction;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.autonomous.AutoRoutine;
import frc.robot.autonomous.TrajectoryBuilder;
import frc.robot.commands.AngleTowerSetpoint;
import frc.robot.commands.AutoRunShooter;
import frc.robot.commands.DeployIntake;
import frc.robot.commands.RetractIntake;
import frc.robot.commands.ReverseConveyorTimed;
import frc.robot.commands.RunConveyorTimed;
import frc.robot.commands.RunIntakeAndConveyor;
import frc.robot.commands.RunShooterPID;
import frc.robot.commands.RunShooterPIDToSetpoint;
import frc.robot.commands.HomeTower;
import frc.robot.commands.HomeTowerAndRetractIntake;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Tower;

public class FourBalls implements AutoRoutine{

    private Intake mIntake = Intake.getInstance();
    private Shooter mShooter = Shooter.getInstance();
    private Conveyor mConveyor = Conveyor.getInstance();
    private Tower mTower = Tower.getInstance();
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
            new InstantCommand(() -> {
                mIntake.stopRoller();
            }, mIntake),
            new ReverseConveyorTimed(0.25),
            new RunShooterPIDToSetpoint(60),
            new ParallelRaceGroup(
                new RunShooterPID(60),
                new SequentialCommandGroup(
                    new RunConveyorTimed(0.5),
                    new WaitCommand(0.3),
                    new RunConveyorTimed(0.5)   
                )
            ),
            new InstantCommand(() -> {
                mShooter.stopShooter();
            }, mShooter),
            new ParallelCommandGroup(
                RamseteCommandBuilder.apply(trajectoryBuilder.getTrajectory("Blue-B2-T"), false),
                new RunIntakeAndConveyor()
            ),
            new WaitCommand(1),
            RamseteCommandBuilder.apply(trajectoryBuilder.getTrajectory("Blue-T-Shoot"), false),
            new InstantCommand(mIntake::stopRoller, mIntake),
            new ReverseConveyorTimed(0.25),
            new RunShooterPIDToSetpoint(60),
            new ParallelRaceGroup(
                new RunShooterPID(60),
                new RunConveyorTimed(1)
                ),
                new InstantCommand(()->{
                mShooter.stopShooter();
            }, mShooter),
            new HomeTowerAndRetractIntake()
            
        );
    }
    
}
