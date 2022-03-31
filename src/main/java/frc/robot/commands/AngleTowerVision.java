package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.PyCamera;
import frc.robot.subsystems.Tower;
import frc.robot.subsystems.VisionSubscriber;

public class AngleTowerVision extends CommandBase implements VisionSubscriber {

    private Tower mTower = Tower.getInstance();
    private PyCamera pycam = PyCamera.getInstantce();
    private Timer mCutoff = new Timer();
    private double mXField = 0;

    public AngleTowerVision() {
        addRequirements(mTower);
        pycam.subscribe(this);
    }

    @Override
    public void initialize() {
        mCutoff.reset();
        mCutoff.start();
    }

    @Override
    public void execute() {
        aimTarget();
    }

    @Override
    public boolean isFinished() {
        return isAimed() || mCutoff.hasElapsed(3);
    }

    public boolean isAimed() {
        return Math.abs(mTower.getTowerAngle() - computeTargetAngle()) < 1.0;
    }

    public double computeTargetAngle() {
        double targetAngle = 67.6818 - 0.101674 * mXField;
        if (targetAngle > 79) {
        return 79;
        }
        return targetAngle;
    }

    public void aimTarget()  {
        mTower.setTowerAngle(computeTargetAngle());
    }

    @Override
    public void handleNewValue(double x_field) {
        mXField = x_field;
    }
    
}
