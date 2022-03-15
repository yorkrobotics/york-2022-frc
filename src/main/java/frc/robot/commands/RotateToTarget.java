package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.PyCamera;

public class RotateToTarget extends PIDCommand {

    private final DriveTrain drive;
    private final PyCamera pycam;

    private double targetAngle = 0;

    public RotateToTarget(DriveTrain drive, PyCamera pycam) {
        super(
            new PIDController(0.3, 0, 0),
            () -> 0.,
            () -> 0.,
            (v) -> {},
            drive, pycam
        );

        this.drive = drive;
        this.pycam = pycam;

        m_useOutput = this::useOutput;
        m_measurement = drive::getGyroAngle;
        m_setpoint = this::getTargetAngle;

        Shuffleboard.getTab("Vision Turning").add(m_controller);

        // Install a listener for the translation_vector entry in the Vision subtable
        NetworkTable visionTable = NetworkTableInstance.getDefault().getTable("Vision");
        // NetworkTableEntry translationVectorEntry = visionTable.getEntry("translation_vector");
        // translationVectorEntry.addListener((ev) -> {
        //     targetAngle = getNewAngle();
        // }, EntryListenerFlags.kUpdate);
    }

    private double getTargetAngle() {
        return targetAngle;
    }

    private double getNewAngle() {
        // Compute the shifted angle as the current angle + the computed angle error
        return drive.getGyroAngle() + pycam.getHorizontalAngle();
    }

    @Override
    public void execute() {
        targetAngle = getNewAngle();
        super.execute();
    }
    private void useOutput(double output) {
        // Saturate the output between -1 and 1
        if (output > 1) output = 1;
        else if (output < -1) output = -1;

        // Rescale the output
        output *= 0.1;

        // Apply the output to the drivetrain
        drive.driveOpenloop(output, -output);
    }
    
}
