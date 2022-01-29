package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.GearShift;

public class GearShiftDown extends CommandBase {
    GearShift m_gear;

    public GearShiftDown(GearShift gear){
        m_gear = gear;
    }

    @Override
    public void execute() {
        m_gear.shiftDown();
    }
}