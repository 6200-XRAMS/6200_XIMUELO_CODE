package frc.robot.commands.Elevador_Cmds;

import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator_Sub.ElevatorSubsystem;

public class ElevatorCmd extends Command {
  
  private final ElevatorSubsystem elevatorSubsystem;
  public DoubleSupplier lspeed;
  public DoubleSupplier rspeed;

 public ElevatorCmd(ElevatorSubsystem elevatorSubsystem, DoubleSupplier lspeed, DoubleSupplier rspeed){
    this.lspeed = lspeed;
    this.elevatorSubsystem = elevatorSubsystem;
    this.rspeed = rspeed;
    addRequirements(elevatorSubsystem);
 }
 
 @Override
  public void initialize() {}

  @Override
  public void execute() {
    double lspeeed = lspeed.getAsDouble();
    double rspeeed = rspeed.getAsDouble();
    elevatorSubsystem.setLMotor(lspeeed);
    elevatorSubsystem.setRMotor(rspeeed);
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}