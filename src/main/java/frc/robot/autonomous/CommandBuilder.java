package frc.robot.autonomous;

import java.util.HashSet;
import java.util.Set;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.autonomous.routines.BlueOneS1B1;
import frc.robot.autonomous.routines.BlueOneS2B2;
import frc.robot.autonomous.routines.BlueOneS3B3;
import frc.robot.autonomous.routines.DoNothing;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

/**
 * Inspired by team 6357
 * credit: github.com/frc6357/robot_code_2022
 */
public class CommandBuilder {

    private Set<AutoRoutine> mAutoRoutines = new HashSet<AutoRoutine>();
    private AutoRoutine doNothing = new DoNothing();

    private Intake mIntake;
    private Shooter mShooter;

    public CommandBuilder(Intake intake, Shooter shooter){

        mIntake = intake;
        mShooter = shooter;

        mAutoRoutines.add(new BlueOneS1B1(mIntake, mShooter));
        mAutoRoutines.add(new BlueOneS2B2());
        mAutoRoutines.add(new BlueOneS3B3());

    }

    public void displayRoutines(SendableChooser<AutoRoutine> sChooser){

        sChooser.setDefaultOption(doNothing.getName(), doNothing);

        for (AutoRoutine routine:mAutoRoutines){
            sChooser.addOption(routine.getName(), routine);
        }
    }
    
}
