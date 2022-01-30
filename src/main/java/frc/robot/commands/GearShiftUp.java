package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.GearShift;

public class GearShiftUp extends CommandBase {

    /**creates a new GearShift */
    GearShift m_gear;
    DriveTrain m_drive;
    boolean isShifted;

    public GearShiftUp(GearShift gear, DriveTrain drive){
        m_gear = gear;
        m_drive = drive;
        isShifted = false;
        //added requirements
        addRequirements(m_gear);
        addRequirements(m_drive);
    }

    @Override
    public void execute() {
        m_gear.shiftUp();
        m_drive.setToHighGear();
        isShifted = true;
    }

    @Override
    public boolean isFinished() {
        return isShifted;
    }
}
