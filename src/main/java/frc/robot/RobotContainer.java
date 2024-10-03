// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.hardware.TalonFX;

import frc.robot.Constants.OperatorConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.passthrough;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Telemetry;
import frc.robot.subsystems.shooter;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */

public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private double MaxSpeed = TunerConstants.kSpeedAt12VoltsMps; // kSpeedAt12VoltsMps desired top speed
  private double MaxAngularRate = 1 * Math.PI; 
  /*
   *  3/4 of a rotation per second max angular velocity (ORIGINAL AT 1.5 Multiplier)
   *  New multiplier set at 1 for a slower rotational speed
   */


  // /* Setting up bindings for necessary control of the swerve drive platform */
  // private final CommandJoystick joystick1 = new CommandJoystick(1); // My joystick
  private final CommandXboxController joystick1 = new CommandXboxController(0);
  private final CommandXboxController joystick2 = new CommandXboxController(1);
  private final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain;
  private final shooter m_shooter = new shooter(new TalonFX(50), new TalonFX(51));

  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
                                                               // driving in open loop
  private final Telemetry logger = new Telemetry(MaxSpeed);
  private final passthrough m_passthrough = new passthrough(new TalonFX(40));
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
     drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
        drivetrain.applyRequest(() -> drive.withVelocityX(joystick1.getLeftY() * MaxSpeed) // Drive forward with
                                                                                           // negative Y (forward)
            .withVelocityY(joystick1.getLeftX() * MaxSpeed) // Drive left with negative X (left)
            .withRotationalRate(joystick1.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
        ));

    if (Utils.isSimulation()) {
      drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    }
    drivetrain.registerTelemetry(logger::telemeterize);

    // PASSTHROUGH BINDINGS
    joystick2.a().onTrue(
      Commands.runOnce(() -> {
        m_passthrough.setspeed(0.25);
      }, m_passthrough)
    );
    
    joystick2.a().onFalse(
      Commands.runOnce(() -> {
        m_passthrough.setspeed(0.0);
      }, m_passthrough)
    );

    joystick2.b().onTrue(
      Commands.runOnce(() -> {
        m_passthrough.setspeed(-0.4);
      }, m_passthrough)
    );
    
    joystick2.b().onFalse(
      Commands.runOnce(() -> {
        m_passthrough.setspeed(0.0);
      }, m_passthrough)
    );

    // SHOOTER BINDINGS
    joystick2.rightTrigger().onTrue(
      Commands.runOnce(() -> {
        m_shooter.shootit();
      }, m_shooter)
    );

    joystick2.rightTrigger().onFalse(
      Commands.runOnce(() -> {
        m_shooter.stop();
      }, m_shooter)
    );

    joystick2.leftTrigger().onTrue(
      Commands.runOnce(() -> {
        m_shooter.eatit();
      }, m_shooter)
    );

    joystick2.leftTrigger().onFalse(
      Commands.runOnce(() -> {
        m_shooter.stop();
      }, m_shooter)
    );
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  // public Command getAutonomousCommand() {
  //   // An example command will be run in autonomous
  //   return Autos.exampleAuto(m_exampleSubsystem);
  // }
}
