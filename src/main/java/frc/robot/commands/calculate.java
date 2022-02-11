package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.PyCamera;
import frc.robot.subsystems.Shooter;

public class calculate extends CommandBase {
    private Shooter m_shooter;
    double x_input;
    double y_input;
    double v_input;

    public calculate(Shooter s) {
        m_shooter = s;
        addRequirements(m_shooter);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        System.out.println(m_shooter.calculate_trajectory_pos(x_input, y_input, v_input));
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return false;
    }
}
