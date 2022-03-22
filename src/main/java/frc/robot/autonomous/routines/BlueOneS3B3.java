package frc.robot.autonomous.routines;

import java.util.function.BiFunction;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.autonomous.AutoRoutine;
import frc.robot.autonomous.TrajectoryBuilder;

public class BlueOneS3B3 implements AutoRoutine{

    @Override
    public String getName() {
        return "Blue-OneBall-S3-B3";
    }

    @Override
    public Command getCommand(TrajectoryBuilder trajectoryBuilder,
            BiFunction<Trajectory, Boolean, Command> RamseteCommandBuilder) {
        return new SequentialCommandGroup(
            RamseteCommandBuilder.apply(trajectoryBuilder.getTrajectory("Blue-S3-B3"), true)
        );
    }
    
}
