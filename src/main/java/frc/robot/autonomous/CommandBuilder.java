package frc.robot.autonomous;

import java.util.HashSet;
import java.util.Set;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.autonomous.routines.DoNothing;
import frc.robot.autonomous.routines.DriveForwardBackward2m;
import frc.robot.autonomous.routines.TwoBalls2To3;

/**
 * Inspired by team 6357
 * credit: github.com/frc6357/robot_code_2022
 */
public class CommandBuilder {

    private Set<AutoRoutine> mAutoRoutines = new HashSet<AutoRoutine>();

    public CommandBuilder(){

        mAutoRoutines.add(new DriveForwardBackward2m());
        mAutoRoutines.add(new TwoBalls2To3());

    }

    public void displayRoutines(SendableChooser<AutoRoutine> sChooser){

        sChooser.setDefaultOption("Do nothing", new DoNothing());

        for (AutoRoutine routine:mAutoRoutines){
            sChooser.addOption(routine.getName(), routine);
        }
    }
    
}
