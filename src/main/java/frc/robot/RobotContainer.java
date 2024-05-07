// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.Elevador_Cmds.ElevatorCmd;
import frc.robot.commands.Elevador_Cmds.ElevatorPIDCmd;
import frc.robot.commands.Hom_Wris_Cmds.PID_HombroCommand;
import frc.robot.commands.Hom_Wris_Cmds.PID_WristCommand;
import frc.robot.commands.Intake_Cmds.IntakeCommand;
import frc.robot.commands.Shooter_Cmds.PIDShooter_Der;
import frc.robot.commands.Shooter_Cmds.PIDShooter_Izq;
import frc.robot.subsystems.Drivetrain_Sub.CommandSwerveDrivetrain;
import frc.robot.subsystems.Elevator_Sub.ElevatorSubsystem;
import frc.robot.subsystems.Hom_Wris_Sub.Hom_Subsystem;
import frc.robot.subsystems.Hom_Wris_Sub.WristSubsystem;
import frc.robot.subsystems.SensorCol_Sub.ColorSensorSubsystem;
import frc.robot.subsystems.ShoterInt_Sub.IntakeSubsystem;
import frc.robot.subsystems.ShoterInt_Sub.Shooter_Der_Sub;
import frc.robot.subsystems.ShoterInt_Sub.Shooter_Izq_Sub;
import frc.robot.subsystems.Vision.LimelightHelpers;

public class RobotContainer {
  private double MaxSpeed = 5; // Metros por Segundo del Chassis
  private double MaxAngularRate = 1.5 * Math.PI; // Velocidad de Rotacion del Chassis
  private Translation2d center = new Translation2d(0 , 0); //Centro de Rotacion del Chassis

  /*Seleccionador de Autonomos */
  private final SendableChooser<Command> autoChooser;

  /*Controles */
  private final CommandXboxController DRIVER = new CommandXboxController(Constants.OI_CONSTANTS.DRIVER_ID); // Control de Driver
  private final CommandXboxController OPERADOR = new CommandXboxController(Constants.OI_CONSTANTS.OPERADOR_ID); //Control del Operador


  /*Subsistemas */
  private final CommandSwerveDrivetrain drivetrain = SwerveConstants.DriveTrain; // El Drivetrain
  // private final Telemetry logger = new Telemetry(MaxSpeed); //Subsistema de Telemetria
  private final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem(); //Subsistema de Elevador
  private final WristSubsystem wristSubsystem = new WristSubsystem(); // Subsistema de muñeca
  private final Hom_Subsystem hombroSubsystem = new Hom_Subsystem(); //Subsistema de Hombro
  private final ColorSensorSubsystem colorSensorSubsystem = new ColorSensorSubsystem(); //Subsistema de color
  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem(); //Subsistema de intake
  private final Shooter_Der_Sub shooter_Der_Sub = new Shooter_Der_Sub(); //Subsistema de Shooter Der
  private final Shooter_Izq_Sub shooter_Izq_Sub = new Shooter_Izq_Sub(); //Subsistema de Shooter Izq
  

  private final SwerveRequest.FieldCentric fieldCentricDrive = new SwerveRequest.FieldCentric()
  .withDeadband(MaxSpeed * 0.1) // Add a 10% deadband
  .withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
  .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric

  private final SwerveRequest.FieldCentricFacingAngle POINT_AT_SPEAKER = new SwerveRequest.FieldCentricFacingAngle()
  .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
  .withDeadband(MaxSpeed * 0.1)
  .withCenterOfRotation(center);

  private final SwerveRequest.FieldCentricFacingAngle POINT_AT_AMP = new SwerveRequest.FieldCentricFacingAngle()
  .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
  .withDeadband(MaxSpeed * 0.1)
  .withCenterOfRotation(center);

  private final SwerveRequest.FieldCentricFacingAngle AUTO_ALING_SPEAKER = new SwerveRequest.FieldCentricFacingAngle()
  .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
  .withDeadband(MaxSpeed * 0.1)
  .withCenterOfRotation(center);

  /*Para Pruebas de AUTO */
  // private Command runAuto = drivetrain.getAutoPath("3NA-(Center)");

  private void configureBindings() {
    
    /*<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<DRIVER>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> */

    /*Comando que aplica velocidades al SWERVE*/
    drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
    drivetrain.applyRequest(() -> fieldCentricDrive.withVelocityX(-DRIVER.getLeftY() * MaxSpeed)
        .withVelocityY(-DRIVER.getLeftX() * MaxSpeed) 
        .withRotationalRate(-DRIVER.getRightX() * MaxAngularRate) 
    ));

    /*Y - Nuevo Enfrente */
    DRIVER.y().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));

    /*RightBumper - Alinear con el eje y (AMP) */
    DRIVER.rightBumper().whileTrue(drivetrain.applyRequest(() -> POINT_AT_AMP.withVelocityX(-DRIVER.getLeftY() * 2)
    .withVelocityY(-DRIVER.getLeftX() * 2)
    .withTargetDirection(Rotation2d.fromDegrees(drivetrain.get_AMP_Allience_Setpoint()))));

    /*LeftBumper - Alinear con el Apriltag (SPEAKER) */
    DRIVER.leftBumper().whileTrue(drivetrain.applyRequest(() -> AUTO_ALING_SPEAKER.withVelocityX(-DRIVER.getLeftY() * 2)
    .withVelocityY(-DRIVER.getLeftX() * 2)
    .withTargetDirection(Rotation2d.fromDegrees(drivetrain.desired_Angle()))));


    /*<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> */

    /*<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<OPERADOR>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> */

    elevatorSubsystem.setDefaultCommand(new ElevatorCmd(elevatorSubsystem,
     () -> -OPERADOR.getLeftY(),
     () -> -OPERADOR.getRightY()
     ));

    // /*POV 0 y 180 - Posiciones del Elevador */
    OPERADOR.pov(0).toggleOnTrue(new ElevatorPIDCmd(elevatorSubsystem, 37));
    OPERADOR.pov(90).toggleOnTrue(
      new SequentialCommandGroup(
      new ElevatorPIDCmd(elevatorSubsystem, 0).withTimeout(1.2),
      new InstantCommand(() -> elevatorSubsystem.setBrakeMode())));

    // /*LeftBumper - Intake a fuerza/Disparar */
    OPERADOR.leftBumper().whileTrue(new InstantCommand(() -> intakeSubsystem.setMotors(Constants.INTAKE_CONSTANTS.INTAKING_VELOCITY)));
    OPERADOR.leftBumper().whileFalse(new InstantCommand(() -> intakeSubsystem.setMotors(0)));

    /*RightBumper - Modo INTAKE*/
    OPERADOR.rightBumper().onTrue(new ParallelCommandGroup(
      new PID_WristCommand(wristSubsystem, Constants.WRIST_CONSTANTS.INTAKING_POSE
      ,Constants.WRIST_CONSTANTS.kP,
      Constants.WRIST_CONSTANTS.kI,
      Constants.WRIST_CONSTANTS.kD),
      new IntakeCommand(intakeSubsystem, colorSensorSubsystem, Constants.INTAKE_CONSTANTS.INTAKING_VELOCITY)
    ));
    
    // /*B - Modo Reposo*/
      OPERADOR.b().onTrue(new ParallelCommandGroup(
      new PID_WristCommand(wristSubsystem, 
      Constants.WRIST_CONSTANTS.REPOSO_POSE,
      Constants.WRIST_CONSTANTS.kP,
      Constants.WRIST_CONSTANTS.kI,
      Constants.WRIST_CONSTANTS.kD),
      new InstantCommand(() -> LimelightHelpers.setLEDMode_ForceOff("limelight")),
      new PID_HombroCommand(hombroSubsystem, 0,
      Constants.HOMBRO_CONTSTANTS.kP_C,0,0)
      ));

    // /*Y - AMP MODE */
     OPERADOR.y().onTrue(new ParallelCommandGroup(
      new PID_WristCommand(wristSubsystem, Constants.WRIST_CONSTANTS.REPOSO_POSE,
      Constants.WRIST_CONSTANTS.kP,
      Constants.WRIST_CONSTANTS.kI,
      Constants.WRIST_CONSTANTS.kD),
      new PID_HombroCommand(hombroSubsystem, 110,
      Constants.HOMBRO_CONTSTANTS.kP,
      Constants.HOMBRO_CONTSTANTS.kI,
      Constants.HOMBRO_CONTSTANTS.kD) 
    ));

    // /*A - SHOOTER MODE*/
     OPERADOR.a().whileTrue(new ParallelCommandGroup(
      new PIDShooter_Der(shooter_Der_Sub, Constants.SHOOTER_CONSTANTS.VELOCITYFORSPEAKER_R),
      new PIDShooter_Izq(shooter_Izq_Sub, Constants.SHOOTER_CONSTANTS.VELOCITYFORSPEAKER_L),
      new PID_WristCommand(wristSubsystem, 
      Constants.WRIST_CONSTANTS.SHOOTER_POSE,
      Constants.WRIST_CONSTANTS.kP_S,
      Constants.WRIST_CONSTANTS.kI_S,
      Constants.WRIST_CONSTANTS.kD_S)
    ));

    // X - Regresar Nota
    OPERADOR.x().whileTrue(new InstantCommand(() -> intakeSubsystem.setMotors(-Constants.INTAKE_CONSTANTS.INTAKING_VELOCITY)));
    OPERADOR.x().whileFalse(new InstantCommand(() -> intakeSubsystem.setMotors(0)));

    OPERADOR.pov(270).whileTrue(new ParallelCommandGroup(
      new PIDShooter_Der(shooter_Der_Sub, 20),
      new PIDShooter_Izq(shooter_Izq_Sub, 25),
      new PID_WristCommand(wristSubsystem, 35,
      Constants.WRIST_CONSTANTS.kP,
      Constants.WRIST_CONSTANTS.kI,
      Constants.WRIST_CONSTANTS.kD)
    ));

    /*<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> */
    
    /*Si es simulacion hace otro enfrente como default */
    if (Utils.isSimulation()) {
      drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    }
    
    /*Telemetria*/
    // drivetrain.registerTelemetry(logger::telemeterize);
  }

  public RobotContainer() {

    /*PID constants for aimatSpeaker and aimAtAmp (Faltan Tunear)*/
    LimelightHelpers.setLEDMode_ForceOff("limelight");
    POINT_AT_SPEAKER.HeadingController.setPID(9.5, 0, 1.0);
    POINT_AT_AMP.HeadingController.setPID(9.5, 0.0, 1.0);
    AUTO_ALING_SPEAKER.HeadingController.setPID(5.0, 0, 0.5);

    // NamedCommands.registerCommand("Intake_Position", new PID_WristCommand(wristSubsystem,
    //   Constants.WRIST_CONSTANTS.INTAKING_POSE,
    //   Constants.WRIST_CONSTANTS.kP,
    //   Constants.WRIST_CONSTANTS.kI,
    //   Constants.WRIST_CONSTANTS.kD));
    
    // NamedCommands.registerCommand("Rolers_In", new IntakeCommand(
    //   intakeSubsystem, 
    //   colorSensorSubsystem, 
    //   Constants.INTAKE_CONSTANTS.INTAKING_VELOCITY).withTimeout(1));

    // NamedCommands.registerCommand("Shooter_Position", new PID_WristCommand(wristSubsystem,
    //   145,
    //   Constants.WRIST_CONSTANTS.kP_S,
    //   Constants.WRIST_CONSTANTS.kI_S,
    //   Constants.WRIST_CONSTANTS.kD_S).withTimeout(2));

    // NamedCommands.registerCommand("DER_SHO", new PIDShooter_Der(shooter_Der_Sub, 
    // Constants.SHOOTER_CONSTANTS.VELOCITYFORSPEAKER_R
    // ).withTimeout(2));

    // NamedCommands.registerCommand("IZQ_SHO", new PIDShooter_Der(shooter_Der_Sub, 
    // Constants.SHOOTER_CONSTANTS.VELOCITYFORSPEAKER_L
    // ).withTimeout(2));

    // NamedCommands.registerCommand("SHOOTT", new InstantCommand(() -> intakeSubsystem.setMotors(
    //   Constants.INTAKE_CONSTANTS.INTAKING_VELOCITY
    // )).withTimeout(1));


    NamedCommands.registerCommand("WRS_INTPOSE", new ParallelCommandGroup(
      new PID_WristCommand(wristSubsystem, Constants.WRIST_CONSTANTS.INTAKING_POSE,
      Constants.WRIST_CONSTANTS.kP,
      Constants.WRIST_CONSTANTS.kI,
      Constants.WRIST_CONSTANTS.kD),
      new IntakeCommand(intakeSubsystem, colorSensorSubsystem, Constants.INTAKE_CONSTANTS.INTAKING_VELOCITY)
    ).withTimeout(2));

    NamedCommands.registerCommand("WRS_SHOPOSE", new ParallelCommandGroup(
      new PIDShooter_Der(shooter_Der_Sub, Constants.SHOOTER_CONSTANTS.VELOCITYFORSPEAKER_R),
      new PIDShooter_Izq(shooter_Izq_Sub, Constants.SHOOTER_CONSTANTS.VELOCITYFORSPEAKER_L),
      new PID_WristCommand(wristSubsystem, Constants.WRIST_CONSTANTS.SHOOTER_POSE,
      Constants.WRIST_CONSTANTS.kP_S,
      Constants.WRIST_CONSTANTS.kI_S,
      Constants.WRIST_CONSTANTS.kD_S)).withTimeout(2));

    NamedCommands.registerCommand("SHOOT", new SequentialCommandGroup(
      new WaitCommand(1),
      new InstantCommand(() -> intakeSubsystem.setMotors(Constants.INTAKE_CONSTANTS.INTAKING_VELOCITY))
      ).withTimeout(1));

      NamedCommands.registerCommand("REPOSO", new ParallelCommandGroup(
      new PID_WristCommand(wristSubsystem, 
      Constants.WRIST_CONSTANTS.REPOSO_POSE,
      Constants.WRIST_CONSTANTS.kP,
      Constants.WRIST_CONSTANTS.kI,
      Constants.WRIST_CONSTANTS.kD),
      new InstantCommand(() -> LimelightHelpers.setLEDMode_ForceOff("limelight")
      )).withTimeout(2));




    /*Creating the autobuider and puting a selection box on shuffleboard*/
    autoChooser = AutoBuilder.buildAutoChooser();
    Shuffleboard.getTab("Autonomus").add("Auto Chooser",autoChooser);

    configureBindings();
  }



  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
    //return runAuto;
  }
}
