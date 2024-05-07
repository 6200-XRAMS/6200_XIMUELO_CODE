package frc.robot.commands.Shooter_Cmds;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ShoterInt_Sub.Shooter_Izq_Sub;


public class PIDShooter_Izq extends Command {
    
    public final Shooter_Izq_Sub shooter_Izq_Sub;
    public final PIDController pidController;
    public static double set_speed;


    public PIDShooter_Izq (Shooter_Izq_Sub shooter_Izq_Sub, double set_speed) {
        this.shooter_Izq_Sub = shooter_Izq_Sub;
        this.pidController = new PIDController(
        Constants.SHOOTER_CONSTANTS.kP, 
        Constants.SHOOTER_CONSTANTS.kI, 
        Constants.SHOOTER_CONSTANTS.kD);
        pidController.setSetpoint(set_speed);
        addRequirements(shooter_Izq_Sub);
    }

    @Override
  public void initialize() {
    pidController.reset();
  }

  @Override
  public void execute() {
    double speed = pidController.calculate(shooter_Izq_Sub.metrosPorSegundo());
    shooter_Izq_Sub.outputSpeed(speed);
  }

  @Override
  public void end(boolean interrupted) {
    shooter_Izq_Sub.outputSpeed(0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }


}