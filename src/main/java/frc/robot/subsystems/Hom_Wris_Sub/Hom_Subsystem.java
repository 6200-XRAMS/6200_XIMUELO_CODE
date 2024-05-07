package frc.robot.subsystems.Hom_Wris_Sub;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class Hom_Subsystem extends SubsystemBase  {

  static double goal;

  public final static WPI_TalonFX M_HOMBRODER_FX = new WPI_TalonFX(Constants.HOMBRO_CONTSTANTS.HOMBRO_DER_ID);
  public final static WPI_TalonFX M_HOMBROIZQ_FX = new WPI_TalonFX(Constants.HOMBRO_CONTSTANTS.HOMBRO_IZQ_ID);

  /*Conversion de valores */
  public static double ticks2AnguloBrazo = M_HOMBROIZQ_FX.getSelectedSensorPosition() * 90 / 69519;
  
  public Hom_Subsystem() {

    // // M_HOMBRODER_FX.configReverseSoftLimitThreshold(0,10);
    // // M_HOMBRODER_FX.configForwardSoftLimitThreshold(45249,10);

    // M_HOMBRODER_FX.configForwardSoftLimitEnable(false);
    // M_HOMBRODER_FX.configReverseSoftLimitEnable(false);

    
    M_HOMBRODER_FX.set(ControlMode.Position, goal);

    M_HOMBRODER_FX.configFactoryDefault();
    M_HOMBRODER_FX.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 30);
    M_HOMBRODER_FX.setInverted(true);

    M_HOMBROIZQ_FX.configFactoryDefault();
    M_HOMBROIZQ_FX.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 30);
    M_HOMBROIZQ_FX.setInverted(false);

  }



  public double getMeasurement() {
    return M_HOMBROIZQ_FX.getSelectedSensorPosition() * 90 / 63827;
  }

  @Override
  public void periodic() {
        SmartDashboard.putNumber("Valor del Hombro", getMeasurement());
    }

  public void setMotors(double speed){
    M_HOMBROIZQ_FX.set(speed);
    M_HOMBRODER_FX.set(speed);
  }

  @Override
  public void simulationPeriodic() {
    
  }
}