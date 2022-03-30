package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Shooter;

public class RunShooterPID extends PIDCommand {

    public RunShooterPID(double velocity) {
        super(
            new PIDController(Constants.kP_VELOCITY_SHOOTER, Constants.kI_VELOCITY_SHOOTER, Constants.kD_VELOCITY_SHOOTER),
            Shooter.getInstance()::getSpeed,
            velocity,
            Shooter.getInstance()::runShooter,
            Shooter.getInstance()
        );
    }
    
}
