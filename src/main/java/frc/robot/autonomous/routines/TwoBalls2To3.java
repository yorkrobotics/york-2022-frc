package frc.robot.autonomous.routines;

import java.util.function.BiFunction;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.autonomous.AutoRoutine;
import frc.robot.autonomous.TrajectoryBuilder;

public class TwoBalls2To3 implements AutoRoutine{

    @Override
    public String getName() {
        return "TwoBalls2To3";
    }

    @Override
    public Command getCommand(TrajectoryBuilder trajectoryBuilder,
            BiFunction<Trajectory, Boolean, Command> RamseteCommandBuilder) {
        return new SequentialCommandGroup(
            RamseteCommandBuilder.apply(trajectoryBuilder.getTrajectory("StartToBall2"), true),
            RamseteCommandBuilder.apply(trajectoryBuilder.getTrajectory("Ball2ToBall3"), false)
        );
    }
    
}
