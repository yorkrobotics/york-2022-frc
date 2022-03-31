package frc.robot.commands;

public class RunShooterPIDToSetpoint extends RunShooterPID{

    public RunShooterPIDToSetpoint(double velocity) {
        super(velocity);
    }
    
    @Override
    public boolean isFinished() {
        return m_controller.atSetpoint();
    }
}
