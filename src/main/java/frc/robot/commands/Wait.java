package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class Wait extends CommandBase {
    
    private double time;
    private Timer timer = new Timer();

    public Wait(double time) {
        this.time = time;
    }

    @Override
    public void initialize() {
        timer.reset();
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(time);
    }

}
