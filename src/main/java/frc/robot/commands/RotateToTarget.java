package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.PyCamera;
import frc.robot.subsystems.VisionSubscriber;

public class RotateToTarget extends PIDCommand implements VisionSubscriber {

    private final DriveTrain drive;
    private final PyCamera pycam;

    private double targetAngle = 0;

    public RotateToTarget(DriveTrain drive, PyCamera pycam) {
        super(
            new PIDController(Constants.kP_VISION_YAW, Constants.kI_VISION_YAW, 0),
            () -> 0.,
            () -> 0.,
            (v) -> {},
            drive, pycam
        );

        this.drive = drive;
        this.pycam = pycam;

        m_controller.setTolerance(0.5, 3);
        m_useOutput = this::useOutput;
        m_measurement = drive::getGyroAngle;
        m_setpoint = this::getTargetAngle;

        Shuffleboard.getTab("Vision Turning").add(m_controller);
        pycam.subscribe(this);
    }


    @Override
    public void initialize() {
        m_controller.reset();
    }

    @Override
    public void execute() {
        super.execute();
    }

    private PIDController getPIDController(){
        switch (DriveTrain.getInstance().getGearMode()){
            case HIGH_GEAR:
                return new PIDController(0, 0, 0);
            case LOW_GEAR:
                return new PIDController(Constants.kP_VISION_YAW, Constants.kI_VISION_YAW, 0);
            case UNKNOWN:
                return null;
            default:
                return null;

        }
    }

    private double getTargetAngle() {
        return targetAngle;
    }

    private double getNewAngle() {
        // Compute the shifted angle as the current angle + the computed angle error
        return drive.getGyroAngle() + pycam.getHorizontalAngle();
    }

    private void useOutput(double output) {
        // Saturate the output between -1 and 1
        if (output > 1) output = 1;
        else if (output < -1) output = -1;

        // Rescale the output
        output *= 0.3;

        // Apply the output to the drivetrain
        drive.driveOpenloop(output, -output);
        SmartDashboard.putNumber("tun to taget output", output);
    }

    @Override
    public boolean isFinished() {
        return m_controller.atSetpoint();
    }

    @Override
    public void handleNewValue(double x) {
        targetAngle = getNewAngle();
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
    }
    
}
