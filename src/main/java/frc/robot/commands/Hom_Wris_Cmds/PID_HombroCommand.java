package frc.robot.commands.Hom_Wris_Cmds;

// import frc.robot.Constants.WristConstants;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Hom_Wris_Sub.Hom_Subsystem;

  
public class PID_HombroCommand extends Command {

private final Hom_Subsystem hom_Subsystem;
private final PIDController pidController;
public double setpoint;
public double kP;
public double kI;
public double kD;


public PID_HombroCommand(Hom_Subsystem hom_Subsystem, double setpoint,
double kP, double kI, double kD){
  this.hom_Subsystem = hom_Subsystem;
  this.kP = kP;
  this.kI = kI;
  this.kD = kD;

  this.pidController = new PIDController(
    kP, kI, kD
  );

  


  pidController.setSetpoint(setpoint);
  addRequirements(hom_Subsystem);
}

  @Override
  public void initialize() {
    pidController.reset();
  }
  
  @Override
  public void execute() {
    double speed = pidController.calculate(hom_Subsystem.getMeasurement());
    hom_Subsystem.setMotors(speed);
  }

  @Override
  public void end(boolean interrupted) {
    hom_Subsystem.setMotors(0);
  }


  @Override
  public boolean isFinished() {
    return false;
  }
}