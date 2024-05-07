package frc.robot.subsystems.ShoterInt_Sub;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Shooter_Izq_Sub extends SubsystemBase {
    
   public final CANSparkMax m_IZQ_MOT = new CANSparkMax(Constants.SHOOTER_CONSTANTS.SHOOTER_IZQ_ID, MotorType.kBrushless);

   public RelativeEncoder IZQ_MOT_RelativeEncoder;

   public static double speed_IZQ_MOT;

   public double metrosPorSegundo () {
      return speed_IZQ_MOT * 25 / 4700;
   }
  
   public Shooter_Izq_Sub() {
      
      m_IZQ_MOT.restoreFactoryDefaults();
      IZQ_MOT_RelativeEncoder = m_IZQ_MOT.getEncoder();

   }

    public void outputSpeed (double R_speed) {
      m_IZQ_MOT.set(R_speed);
   }

   @Override
   public void periodic() {

      speed_IZQ_MOT = IZQ_MOT_RelativeEncoder.getVelocity();

      SmartDashboard.putNumber("Speed Motor 2", metrosPorSegundo());
      SmartDashboard.putNumber("Speed Motor 2 RPM", IZQ_MOT_RelativeEncoder.getVelocity());

     }

}