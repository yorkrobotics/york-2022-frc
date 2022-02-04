package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.GearShift;

public class GearShiftUp extends CommandBase {

    /**creates a new GearShift */
    GearShift m_gear;
    private boolean shifted = false;

    public GearShiftUp(GearShift gear){
        m_gear = gear;
    }

    @Override
    public void execute() {
        m_gear.shiftUp();
        shifted = true;
    }

    @Override
    public boolean isFinished() {
        return shifted;
    }
}
