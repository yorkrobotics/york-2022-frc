package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.GearShift;

public class GearShiftUp extends CommandBase {

    /**creates a new GearShift */
    GearShift m_gear;

    public GearShiftUp(GearShift gear){
        m_gear = gear;
    }

    @Override
    public void execute() {
        m_gear.shiftUp();
    }
}
