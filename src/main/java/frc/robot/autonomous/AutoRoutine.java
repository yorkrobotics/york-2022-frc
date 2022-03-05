package frc.robot.autonomous;

import java.util.function.BiFunction;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * Inspired by team 6357
 * credit: github.com/frc6357/robot_code_2022
 */
public interface AutoRoutine {
    /**
     * A method that returns the name of the AutoRoutine
     * @return The name of the AutoRoutine
     */
    public String getName();

    /**
     * 
     * @param trajectoryBuilder
     *          The class that creates trajectories
     * @return The Auto commmand
     */
    public Command getCommand(TrajectoryBuilder trajectoryBuilder, BiFunction<Trajectory, Boolean, Command> RamseteCommandBuilder);
}
