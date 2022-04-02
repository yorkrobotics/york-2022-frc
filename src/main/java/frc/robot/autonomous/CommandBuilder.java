package frc.robot.autonomous;

import java.util.HashSet;
import java.util.Set;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.autonomous.routines.OneBallTop;
import frc.robot.autonomous.routines.OneBallMid;
import frc.robot.autonomous.routines.OneBallBottom;
import frc.robot.autonomous.routines.DoNothing;
import frc.robot.autonomous.routines.DriveOutStraight;
import frc.robot.autonomous.routines.FourBalls;


/**
 * Inspired by team 6357
 * credit: github.com/frc6357/robot_code_2022
 */
public class CommandBuilder {

    private Set<AutoRoutine> mAutoRoutines = new HashSet<AutoRoutine>();
    private AutoRoutine doNothing = new DoNothing();

    public CommandBuilder(){

        mAutoRoutines.add(new OneBallTop());
        mAutoRoutines.add(new OneBallMid());
        mAutoRoutines.add(new OneBallBottom());
        mAutoRoutines.add(new DriveOutStraight());

        mAutoRoutines.add(new FourBalls());

    }

    public void displayRoutines(SendableChooser<AutoRoutine> sChooser){

        sChooser.setDefaultOption(doNothing.getName(), doNothing);

        for (AutoRoutine routine:mAutoRoutines){
            sChooser.addOption(routine.getName(), routine);
        }
    }
    
}
