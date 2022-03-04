package frc.robot.autonomous;

import java.io.File;
import java.io.IOException;
import java.util.HashMap;
import java.util.Map;


import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.Filesystem;

import edu.wpi.first.wpilibj.DriverStation;


public class TrajectoryBuilder {

    private Map<String, Trajectory>  mTrajectoryMap = new HashMap<String, Trajectory>();

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
