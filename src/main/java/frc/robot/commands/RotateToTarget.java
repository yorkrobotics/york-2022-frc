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
import frc.robot.subsystems.VisionSubscriber;

public class RotateToTarget extends PIDCommand implements VisionSubscriber {

    private final DriveTrain drive;
    private final PyCamera pycam;

    private double targetAngle = 0;
    private boolean isDone = false;

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
        pycam.subscribe(this);
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
        if (Math.abs(pycam.getHorizontalAngle()) < 1 ) {
            isDone = true;
        }
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

    @Override
    public boolean isFinished() {
        return isDone;
    }

    @Override
    public void handleNewValue(double x) {
        targetAngle = getNewAngle();
        // TODO Auto-generated method stub
    }
    
}
