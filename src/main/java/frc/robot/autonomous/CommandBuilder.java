package frc.robot.autonomous;

import java.util.HashSet;
import java.util.Set;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.autonomous.routines.BlueOneS1B1;
import frc.robot.autonomous.routines.BlueOneS2B2;
import frc.robot.autonomous.routines.BlueOneS3B3;
import frc.robot.autonomous.routines.DoNothing;
import frc.robot.autonomous.routines.DriveOutStraight;
import frc.robot.autonomous.routines.FourBalls;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Tower;

/**
 * Inspired by team 6357
 * credit: github.com/frc6357/robot_code_2022
 */
public class CommandBuilder {

    private Set<AutoRoutine> mAutoRoutines = new HashSet<AutoRoutine>();
    private AutoRoutine doNothing = new DoNothing();

    public CommandBuilder(){

        mAutoRoutines.add(new BlueOneS1B1());
        mAutoRoutines.add(new BlueOneS2B2());
        mAutoRoutines.add(new BlueOneS3B3());
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
