package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import frc.robot.Constants;

public class VelocityControlCANSparkMax implements MotorController {

    private CANSparkMax m_motor;
    private SparkMaxPIDController m_controller;
    private RelativeEncoder m_encoder;

    public VelocityControlCANSparkMax(CANSparkMax motor) {
        m_motor = motor;
        m_encoder = motor.getEncoder();
        m_controller = motor.getPIDController();
    }
    

    @Override
    public void set(double speed) {
        m_controller.setReference(speed * Constants.MAX_RPM, CANSparkMax.ControlType.kVelocity);
    }

    @Override
    public double get() {
        return m_encoder.getVelocity() / Constants.MAX_RPM;
    }

    @Override
    public void setInverted(boolean isInverted) {
        m_motor.setInverted(isInverted);
        
    }

    @Override
    public boolean getInverted() {
        return m_motor.getInverted();
    }

    @Override
    public void disable() {
        m_motor.disable();
        
    }

    @Override
    public void stopMotor() {
        m_motor.set(0);
        
    }

    
    public void setP(double gain) {
        m_controller.setP(gain);
    }

    public void setI(double gain) {
        m_controller.setI(gain);
    }

    public void setD(double gain) {
        m_controller.setD(gain);
    }

    public void setIZone(double IZone) {
        m_controller.setIZone(IZone);
    }

    public void setFF(double gain) {
        m_controller.setFF(gain);
    }

    public void setOutputRange(double min, double max) {
        m_controller.setOutputRange(min, max);
    }
    
    public CANSparkMax getMotor(){
        return m_motor;
    }

    public RelativeEncoder getEncoder(){
        return m_encoder;
    }

    public void follow(VelocityControlCANSparkMax vControl){
        m_motor.follow(vControl.getMotor());
    }

    public void setPosition(double value){
        m_controller.setReference(value, CANSparkMax.ControlType.kPosition);
    }
    


}
