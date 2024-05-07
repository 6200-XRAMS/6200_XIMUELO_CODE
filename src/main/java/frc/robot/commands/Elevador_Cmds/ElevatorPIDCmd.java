package frc.robot.commands.Elevador_Cmds;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Elevator_Sub.ElevatorSubsystem;

public class ElevatorPIDCmd extends Command {
  
  private final ElevatorSubsystem elevatorSubsystem;
  private final PIDController pidController;
  public double setPoint;
 
  
 public ElevatorPIDCmd(ElevatorSubsystem elevatorSubsystem, double setPoint){
    this.elevatorSubsystem = elevatorSubsystem;
    //en cuanto se cambie la regla de tres en el subsystem, probar hasta hallar valores adecuados
    //aun no se sabe qué altura ni como entonces ps si
    this.pidController = new PIDController(Constants.ELEVADOR_CONSTANTS.kP, Constants.ELEVADOR_CONSTANTS.kI, Constants.ELEVADOR_CONSTANTS.kD);
    pidController.setSetpoint(setPoint);
    addRequirements(elevatorSubsystem);
 }
 
 @Override
  public void initialize() {
    pidController.reset();
    elevatorSubsystem.setCoastMode();

  }

  @Override
  public void execute() {
    double speed = pidController.calculate(elevatorSubsystem.getEncodertocm_Left());
    elevatorSubsystem.setBMotors(speed);
  }

  @Override
  public void end(boolean interrupted) {
    elevatorSubsystem.setBMotors(0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}