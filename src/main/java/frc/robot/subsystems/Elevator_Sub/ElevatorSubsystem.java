package frc.robot.subsystems.Elevator_Sub;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ElevatorSubsystem extends SubsystemBase {
  
   private final WPI_TalonFX m_Elevador_L = new WPI_TalonFX(Constants.ELEVADOR_CONSTANTS.ELEVADOR_LEFT_ID);
   private final WPI_TalonFX m_Elevador_R = new WPI_TalonFX(Constants.ELEVADOR_CONSTANTS.ELEVADOR_RIGHT_ID);
   
   public void cofigMotors(){
      /*Izquierda */
      m_Elevador_L.setInverted(true);
      m_Elevador_L.follow(m_Elevador_R);
      m_Elevador_L.configForwardSoftLimitThreshold(140607, 10);
      m_Elevador_L.configReverseSoftLimitThreshold(0, 10);
      m_Elevador_L.configForwardSoftLimitEnable(true);
      m_Elevador_L.configReverseSoftLimitEnable(true);

      /*Derecha */
      m_Elevador_R.configForwardSoftLimitThreshold(148029, 10);
      m_Elevador_R.configReverseSoftLimitThreshold(0, 10);
      m_Elevador_R.configForwardSoftLimitEnable(true);
      m_Elevador_R.configReverseSoftLimitEnable(true);
      

  }

  public ElevatorSubsystem(){
      m_Elevador_L.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor,0,10);
      m_Elevador_R.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor,0,10);
      cofigMotors();

  }

  public void setBrakeMode(){
    m_Elevador_L.setNeutralMode(NeutralMode.Brake);
    m_Elevador_R.setNeutralMode(NeutralMode.Brake);
  }

   public void setCoastMode(){
    m_Elevador_L.setNeutralMode(NeutralMode.Coast);
    m_Elevador_R.setNeutralMode(NeutralMode.Coast);
  }


  public double getEncodertocm_Left(){
    //cambiar esto en cuanto se pueda (la regla de 3)
    return m_Elevador_L.getSelectedSensorPosition() * 37 / 140607;
  }
  public double getEncodertocm_Right(){
    return m_Elevador_R.getSelectedSensorPosition() * 37 / 148029;
  }

  @Override
  public void periodic() {
        SmartDashboard.putNumber("Height_Left",getEncodertocm_Left());
        SmartDashboard.putNumber("Height_Right",getEncodertocm_Right());
  }

  public void setLMotor(double speed) {
    m_Elevador_L.set(speed);
  }
  public void setRMotor(double speed) {
    m_Elevador_R.set(speed);
  }
  public void setBMotors(double speed) {
    m_Elevador_L.set(speed);
    m_Elevador_R.set(speed);
    
  }

  @Override
  public void simulationPeriodic() {}
}