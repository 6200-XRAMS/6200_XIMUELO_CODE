package frc.robot.commands.Hom_Wris_Cmds;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Hom_Wris_Sub.WristSubsystem;
// import frc.robot.subsystems.Vision.LimelightHelpers;


public class PID_Wrist_ajustable extends Command {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final WristSubsystem wristSubsystem;
    public double setpoint;
    // private double lastvalue;


    /**
     * @param subsystem The subsystem used by this command.
     */
  
    public PID_Wrist_ajustable(WristSubsystem wristSubsystem) {
      this.wristSubsystem = wristSubsystem;
      addRequirements(wristSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        wristSubsystem.pid_Reset();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        
        // if ((LimelightHelpers.getTV("limelight"))){
        //     /*Realiza un calculo basico de trigonometria, tomando la altura del speaker y la lejania del robot para asi obtener el angulo deseado*/
        //      //setpoint = Math.toDegrees(Math.atan(Constants.Targetvalues_Angulo.speaker_point_height / photonLL.getXDistance()));
        //      setpoint = ((-4.074552 * Math.pow(
        //         -LimelightHelpers.getCameraPose3d_TargetSpace("limelight").getZ(), 3)) +
        //         (32.4296 * Math.pow(-LimelightHelpers.getCameraPose3d_TargetSpace("limelight").getZ(), 2)) + 
        //         (-92.2537 * -LimelightHelpers.getCameraPose3d_TargetSpace("limelight").getZ()) + 207.463);
        //     lastvalue = setpoint;
        //   } else {
        //      setpoint = lastvalue;
        //   }
    
        wristSubsystem.set_to_target(setpoint);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        wristSubsystem.setMotors(0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
