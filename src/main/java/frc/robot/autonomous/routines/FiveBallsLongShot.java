package frc.robot.autonomous.routines;

import java.util.function.BiFunction;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.autonomous.AutoRoutine;
import frc.robot.autonomous.TrajectoryBuilder;
import frc.robot.commands.AngleTowerSetpoint;
import frc.robot.commands.DeployIntake;
import frc.robot.commands.ReverseConveyorTimed;
import frc.robot.commands.RunConveyorTimed;
import frc.robot.commands.RunIntakeAndConveyor;
import frc.robot.commands.RunShooterPID;
import frc.robot.commands.RunShooterPIDToSetpoint;
import frc.robot.commands.ShootBallSequence;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class FiveBallsLongShot implements AutoRoutine{

    private final double TERMINAL_WAIT_TIME = 5;

    private Intake mIntake = Intake.getInstance();
    private Shooter mShooter = Shooter.getInstance();
    
    @Override
    public String getName() {
        return "FiveBalls-LongShot";
    }

    @Override
    public Command getCommand(TrajectoryBuilder trajectoryBuilder,
            BiFunction<Trajectory, Boolean, Command> RamseteCommandBuilder) {
        return new SequentialCommandGroup(
            new ParallelCommandGroup(
                RamseteCommandBuilder.apply(trajectoryBuilder.getTrajectory("Blue-S3-B3"), true),
                new SequentialCommandGroup(
                    new DeployIntake(),
                    new RunIntakeAndConveyor(),
                    new AngleTowerSetpoint(60)
                )
            ),

            new WaitCommand(0.5),
            new ShootBallSequence(48),
            new InstantCommand(() -> {
                mShooter.stopShooter();
                mIntake.stopRoller();
            }, mShooter, mIntake),

            RamseteCommandBuilder.apply(trajectoryBuilder.getTrajectory("Blue-B3-Turn"), false),
            new ParallelCommandGroup(
                RamseteCommandBuilder.apply(trajectoryBuilder.getTrajectory("Blue-Turn-B2"), false),
                new RunIntakeAndConveyor()
            ),

            new ParallelCommandGroup(
                RamseteCommandBuilder.apply(trajectoryBuilder.getTrajectory("Blue-B2-T"), false),
                new RunIntakeAndConveyor(),
                new AngleTowerSetpoint(52)
            ),

            new ReverseConveyorTimed(0.25),
            new RunShooterPIDToSetpoint(65),
            new ParallelRaceGroup(
                new RunShooterPID(65),
                new RunConveyorTimed(3)
            ),

            new InstantCommand(() -> {
                mShooter.stopShooter();
                mIntake.stopRoller();
            }, mShooter, mIntake)

            // new HomeTowerAndRetractIntake()

        );
    }
    
}
