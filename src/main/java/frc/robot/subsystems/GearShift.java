package frc.robot.subsystems;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
//import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;


public class GearShift extends SubsystemBase {
    //private XboxController m_controller;
    private DoubleSolenoid m_shifter;


    public void robotInit() {
        m_shifter = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 6, 1);
       // m_controller = new XboxController(0); 
    }
    public void shiftDown(){
        m_shifter.set(Value.kForward);
    }
    public void shiftUp(){
        m_shifter.set(Value.kReverse);
    }
}

