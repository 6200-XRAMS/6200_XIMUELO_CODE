package frc.robot.subsystems.Hom_Wris_Sub;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.Vision.LimelightHelpers;

public class WristSubsystem extends SubsystemBase {

  //private final CANSparkMax WRIST_MOTOR = new CANSparkMax(Constants.WRIST_CONSTANTS.id, MotorType.kBrushless);
  private final WPI_TalonFX WRIST_MOTOR = new WPI_TalonFX(Constants.WRIST_CONSTANTS.id);

  private final PIDController pidController = new PIDController(
      Constants.WRIST_CONSTANTS.kP_S,
      Constants.WRIST_CONSTANTS.kI_S,
      Constants.WRIST_CONSTANTS.kD_S
  );

 // public RelativeEncoder encoder;

  public double getEncodertodegreesM1(){
   return (WRIST_MOTOR.getSelectedSensorPosition() * 90) / 40852 ;
    //153
  }
  
  
  public WristSubsystem() {

    WRIST_MOTOR.configFactoryDefault();
    WRIST_MOTOR.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 30);
    WRIST_MOTOR.setInverted(true);

    // WRIST_MOTOR.configReverseSoftLimitThreshold(0,10);
    // WRIST_MOTOR.configForwardSoftLimitThreshold(45249,10);

   WRIST_MOTOR.configForwardSoftLimitEnable(false);
    WRIST_MOTOR.configReverseSoftLimitEnable(false);
  }

  @Override
  public void periodic() {
        SmartDashboard.putNumber("Valor de Muneca",getEncodertodegreesM1());
        SmartDashboard.putNumber("DISTANCIA", -LimelightHelpers.getCameraPose3d_TargetSpace("limelight").getZ());
        // SmartDashboard.putNumber("TELEMERIZE", telemeterizew());
    }

  public void setMotors(double speed){
    WRIST_MOTOR.set(speed);
  }


  public void pid_Reset(){
        pidController.reset();
    }

    /*Metodo para que el angulo se establezca */
  public void set_to_target(double angle_deseado){
        pidController.setSetpoint(angle_deseado);
        double speed = pidController.calculate(getEncodertodegreesM1());
        WRIST_MOTOR.set(speed);
    }

  // public double telemeterizew(){
  //   double setpoint;
  //   if ((LimelightHelpers.getTV("limelight"))){
  //     /*Realiza un calculo basico de trigonometria, tomando la altura del speaker y la lejania del robot para asi obtener el angulo deseado*/
  //      //setpoint = Math.toDegrees(Math.atan(Constants.Targetvalues_Angulo.speaker_point_height / photonLL.getXDistance()));
  //      return setpoint = (-4.074552 * Math.pow(
  //       -LimelightHelpers.getCameraPose3d_TargetSpace("limelight").getZ(), 3)) +
  //       (32.4296 * Math.pow(-LimelightHelpers.getCameraPose3d_TargetSpace("limelight").getZ(), 2)) + 
  //       (-92.2537 * -LimelightHelpers.getCameraPose3d_TargetSpace("limelight").getZ()) + 207.463;
  //   } else {
  //      return setpoint = 0;
  //   }
  // }

  @Override
  public void simulationPeriodic() {
    
  }
}