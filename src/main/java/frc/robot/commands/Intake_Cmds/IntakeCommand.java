package frc.robot.commands.Intake_Cmds;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SensorCol_Sub.ColorSensorSubsystem;
import frc.robot.subsystems.ShoterInt_Sub.IntakeSubsystem;
import frc.robot.subsystems.Vision.LimelightHelpers;


public class IntakeCommand extends Command {
  
 private final IntakeSubsystem intakeSubsystem;
 private final ColorSensorSubsystem colorSensorSubsystem;
 private double speed;

 public IntakeCommand(IntakeSubsystem intakeSubsystem, ColorSensorSubsystem colorSensorSubsystem, double speed){
    this.speed = speed;
    this.colorSensorSubsystem = colorSensorSubsystem;
    this.intakeSubsystem = intakeSubsystem;
    addRequirements(intakeSubsystem, colorSensorSubsystem);
 }
 
 
 @Override
  public void initialize() {}

  @Override
  public void execute() {
    intakeSubsystem.setMotors(speed);
  }

  @Override
  public void end(boolean interrupted) {
    intakeSubsystem.setMotors(0);
    LimelightHelpers.setLEDMode_ForceBlink("limelight");
  }

  @Override
  public boolean isFinished() {
    // return false;
    if ((colorSensorSubsystem.colorString == "Yellow") || (colorSensorSubsystem.colorString == "Unknown")) {
      return true;
    } else {
      return false;
    }
    }
}