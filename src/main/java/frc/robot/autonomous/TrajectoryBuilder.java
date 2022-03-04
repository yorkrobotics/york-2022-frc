package frc.robot.autonomous;

import java.io.File;
import java.io.IOException;
import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrain;
import edu.wpi.first.wpilibj.DriverStation;


public class TrajectoryBuilder {

    private Map<String, Trajectory>  mTrajectoryMap = new HashMap<String, Trajectory>();
    private DriveTrain mDrive = new DriveTrain();
    private PIDController mLeftAutoController = new PIDController(Constants.kP_AUTO, 0, 0), 
    mRightAutoController = new PIDController(Constants.kP_AUTO, 0, 0);

    public TrajectoryBuilder (String pathSubDir){

        File pathDir = new File(Filesystem.getDeployDirectory(), pathSubDir);
        File[] pathFolder = pathDir.listFiles();

        for (File pathFile : pathFolder){
            

            if (!pathFile.getName().contains(".json")){
                continue;
            }

            Trajectory trajectory = makeTrajectoryFromJSON(pathFile);
            if (trajectory != null){
                mTrajectoryMap.put(pathFile.getName().replace(".wpilib.json", ""), trajectory);
            }
        }
    }

    public boolean hasTrajectory(String name){
        return mTrajectoryMap.keySet().contains(name);
    }

    public Trajectory getTrajectory(String name){
        return mTrajectoryMap.get(name);
    }

    public Command makeTrajectoryToCommand(Trajectory trajectory, Boolean resetOdometry){
        RamseteCommand command =  new RamseteCommand(
            trajectory, 
            mDrive::getPose, 
            new RamseteController(2.0, 0.7), 
            mDrive.getFeedForward(), 
            mDrive.getKinematics(), 
            mDrive::getWheelSpeeds, 
            mLeftAutoController, 
            mRightAutoController, 
            mDrive::tankDriveVolts, 
            mDrive);
        return resetOdometry ? 
            new SequentialCommandGroup(
                new InstantCommand(()-> mDrive.resetOdometry(trajectory.getInitialPose()), mDrive)).andThen( 
                command.andThen(()->mDrive.tankDriveVolts(0, 0)))
                : command.andThen(()->mDrive.tankDriveVolts(0, 0));
    }

    private Trajectory makeTrajectoryFromJSON(File trajectotyJSON){

        Trajectory trajectory = new Trajectory();

        try {
            trajectory = TrajectoryUtil.fromPathweaverJson(trajectotyJSON.toPath());
        } 

        catch (IOException ex) {
            DriverStation.reportError("Unable to open trajectory: " + trajectotyJSON.getName(), ex.getStackTrace());
            return null;
        }
        return trajectory;
    }
    
}
