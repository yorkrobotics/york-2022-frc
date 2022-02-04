package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.GearShift;

public class GearShiftUp extends CommandBase {

    /**creates a new GearShift */
    GearShift mGear;
    DriveTrain mDrive;
    boolean isShifted;

    public GearShiftUp(GearShift gear, DriveTrain drive){
        mGear = gear;
        mDrive = drive;
        isShifted = false;
        //added requirements
        addRequirements(mGear);
        addRequirements(mDrive);
    }

    @Override
    public void execute() {
        mGear.shiftUp();
        mDrive.setToHighGear();
        isShifted = true;
    }

    @Override
    public boolean isFinished() {
        return isShifted;
    }
}
