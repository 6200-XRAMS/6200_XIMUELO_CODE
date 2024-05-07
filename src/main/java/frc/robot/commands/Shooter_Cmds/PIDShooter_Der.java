package frc.robot.commands.Shooter_Cmds;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ShoterInt_Sub.Shooter_Der_Sub;


public class PIDShooter_Der extends Command {
    
    public final Shooter_Der_Sub shooter_Der_Sub;
    public final PIDController pidController;
    public static double set_speed;


    public PIDShooter_Der (Shooter_Der_Sub shooter_Der_Sub, double set_speed) {
        this.shooter_Der_Sub = shooter_Der_Sub;
        this.pidController = new PIDController(
        Constants.SHOOTER_CONSTANTS.kP, 
        Constants.SHOOTER_CONSTANTS.kI, 
        Constants.SHOOTER_CONSTANTS.kD);
        pidController.setSetpoint(set_speed);
        
        addRequirements(shooter_Der_Sub);
    }

    @Override
  public void initialize() {
    pidController.reset();
  }

  @Override
  public void execute() {
    double speed = pidController.calculate(shooter_Der_Sub.metrosPorSegundo());
    shooter_Der_Sub.outputSpeed(speed);
  }

  @Override
  public void end(boolean interrupted) {
    shooter_Der_Sub.outputSpeed(0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}