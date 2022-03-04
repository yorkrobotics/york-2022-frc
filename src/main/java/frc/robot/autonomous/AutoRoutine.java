package frc.robot.autonomous;

import edu.wpi.first.wpilibj2.command.Command;

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
    public Command getCommand(TrajectoryBuilder trajectoryBuilder);
}
