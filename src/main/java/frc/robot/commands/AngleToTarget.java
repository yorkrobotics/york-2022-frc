package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Tower;

public class AngleToTarget extends CommandBase {

    private Tower mTower;
    private Timer mCutoff = new Timer();

    public AngleToTarget(Tower tower) {
        mTower = tower;
        addRequirements(tower);
    }

    @Override
    public void initialize() {
        mCutoff.reset();
    }

    @Override
    public void execute() {
        mTower.aimTarget();
    }

    @Override
    public boolean isFinished() {
        return mTower.isAimed() || mCutoff.hasElapsed(3);
    }
    
}
