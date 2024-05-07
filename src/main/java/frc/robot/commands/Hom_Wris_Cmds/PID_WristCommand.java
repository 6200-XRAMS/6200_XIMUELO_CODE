package frc.robot.commands.Hom_Wris_Cmds;

// import frc.robot.Constants.WristConstants;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Hom_Wris_Sub.WristSubsystem;

  
public class PID_WristCommand extends Command {

private final WristSubsystem wristSubsystem;
private final PIDController pidController;
public double setpoint;
public double kP;
public double kI;
public double kD;

public PID_WristCommand(WristSubsystem wristSubsystem, double setpoint,
double kP, double kI, double kD){
  this.wristSubsystem = wristSubsystem;
  this.kP = kP;
  this.kI = kI;
  this.kD = kD;

  this.pidController = new PIDController(
    kP, kI, kD
  );
  
  pidController.setSetpoint(setpoint);
  addRequirements(wristSubsystem);
}

  @Override
  public void initialize() {
    pidController.reset();
  }
  
  @Override
  public void execute() {
    double speed = pidController.calculate(wristSubsystem.getEncodertodegreesM1());
    wristSubsystem.setMotors(speed);
  }

  @Override
  public void end(boolean interrupted) {
    wristSubsystem.setMotors(0);
  }


  @Override
  public boolean isFinished() {
    return false;
  }
}