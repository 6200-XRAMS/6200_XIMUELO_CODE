package frc.robot.subsystems.ShoterInt_Sub;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Shooter_Der_Sub extends SubsystemBase {
    
   public final CANSparkMax m_DER_MOT = new CANSparkMax(Constants.SHOOTER_CONSTANTS.SHOOTER_DER_ID, MotorType.kBrushless);

   public RelativeEncoder DER_MOT_RelativeEncoder;

   public static double speed_DER_MOT;

   public double metrosPorSegundo () {
      return speed_DER_MOT * 25 / 4700;
   }
  
   public Shooter_Der_Sub() {
      
      m_DER_MOT.restoreFactoryDefaults();
      DER_MOT_RelativeEncoder = m_DER_MOT.getEncoder();

   }

    public void outputSpeed (double R_speed) {
      m_DER_MOT.set(R_speed);
   }

   @Override
   public void periodic() {

      speed_DER_MOT = DER_MOT_RelativeEncoder.getVelocity();

      SmartDashboard.putNumber("Speed Motor 1", metrosPorSegundo());
      SmartDashboard.putNumber("Speed Motor 1 RPM", DER_MOT_RelativeEncoder.getVelocity());

     }

}