package frc.robot.autonomous.routines;

import java.util.function.BiFunction;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.autonomous.AutoRoutine;
import frc.robot.autonomous.TrajectoryBuilder;

public class DriveForwardBackward2m implements AutoRoutine{

    @Override
    public String getName() {
        return "DriveForwardAndBackward1m";
    }

    @Override
    public Command getCommand(TrajectoryBuilder trajectoryBuilder, BiFunction<Trajectory, Boolean, Command> ramseteCommandBuilder) {
        return new SequentialCommandGroup(
            ramseteCommandBuilder.apply(trajectoryBuilder.getTrajectory("TestDriveForward"), true),
            ramseteCommandBuilder.apply(trajectoryBuilder.getTrajectory("TestDriveBackward"), false)
        );
    }
    
}
