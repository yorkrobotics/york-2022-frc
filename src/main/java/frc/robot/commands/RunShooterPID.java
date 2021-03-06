package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Shooter;

public class RunShooterPID extends PIDCommand {
    public double m_velocity;

    public RunShooterPID(double velocity) {
        super(
            new PIDController(Constants.kP_VELOCITY_SHOOTER, Constants.kI_VELOCITY_SHOOTER, Constants.kD_VELOCITY_SHOOTER),
            Shooter.getInstance()::getSpeed,
            () -> 0,
            (output) -> {},
            Shooter.getInstance()
        );

        m_velocity = velocity;

        m_controller.setTolerance(0.5, 40);
        m_setpoint = this::getVelocity;
        m_useOutput = this::useOutput;

        Shuffleboard.getTab("Shooter").add(this.getController());
    }

    @Override
    public void execute() {
        SmartDashboard.putBoolean("Shooter at setpoint", m_controller.atSetpoint());
        super.execute();
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        Shooter.getInstance().stopShooter();
    }

    @Override
    public void initialize() {
        m_controller.reset();
        super.initialize();
    }

    public void setVelocity(double velocity) {
        m_velocity = velocity;
    }

    public double getVelocity() {
        return m_velocity;
    }

    private void useOutput(double output) {
        Shooter.getInstance().runShooter(output + Shooter.getInstance().getkFF() * getVelocity());
    }
    
}
