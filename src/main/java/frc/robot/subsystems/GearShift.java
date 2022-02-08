package frc.robot.subsystems;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
//import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;


public class GearShift extends SubsystemBase {
    //private XboxController m_controller;
    private DoubleSolenoid m_shifter;


    public GearShift(){
        m_shifter = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 6, 1);
       // m_controller = new XboxController(0); 
    }

  /*  @Override
    public void periodic() {
        super.periodic();
        System.out.print(m_shifter.get());
    }
    */

    public void shiftDown(){
        m_shifter.set(Value.kForward);
        //System.out.print(m_shifter.get());
    }
    public void shiftUp(){
        m_shifter.set(Value.kReverse);
       // System.out.print(m_shifter.get());
    }

}

