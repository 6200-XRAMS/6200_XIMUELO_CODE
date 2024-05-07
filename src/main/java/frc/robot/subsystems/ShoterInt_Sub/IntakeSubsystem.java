package frc.robot.subsystems.ShoterInt_Sub;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {

  private final CANSparkMax m_UP_Intake = new CANSparkMax(Constants.INTAKE_CONSTANTS.MOTOR_UP_INTAKE_ID, MotorType.kBrushless);
  private final CANSparkMax m_Down_Intake = new CANSparkMax(Constants.INTAKE_CONSTANTS.MOTOR_DOWN_INTAKE_ID, MotorType.kBrushless);
  public RelativeEncoder DER_MOT_RelativeEncoder;
  
  public IntakeSubsystem() {
    DER_MOT_RelativeEncoder = m_UP_Intake.getEncoder();

  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Intake On", On(DER_MOT_RelativeEncoder.getVelocity()));


  }

  public boolean On(double speed){
    if (speed >= 0.1){
      return true;
    } else {
      return false;
    }
  }

  public void setMotors(double speed) {
    m_UP_Intake.set(speed);
    m_Down_Intake.set(speed);
  }

  @Override
  public void simulationPeriodic() {}
}