package frc.robot.autonomous.routines;

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
    public Command getCommand(TrajectoryBuilder trajectoryBuilder) {
        return new SequentialCommandGroup(
            trajectoryBuilder.makeTrajectoryToCommand(trajectoryBuilder.getTrajectory("TestDriveForward"), true),
            trajectoryBuilder.makeTrajectoryToCommand(trajectoryBuilder.getTrajectory("TestDriveBackward"), false)
        );
    }
    
}
