// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.NeoMotors;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final NeoMotors m_exampleSubsystem = new NeoMotors();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    SmartDashboard.putData("Shooter", m_exampleSubsystem);
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
   
    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    m_driverController.x().onTrue(m_exampleSubsystem.closedLoopController());
    m_driverController.b().onTrue(m_exampleSubsystem.stopC());
    m_driverController.y().onTrue(m_exampleSubsystem.changeSpeedP());
    m_driverController.a().onTrue(m_exampleSubsystem.changeSpeedN());
    // m_driverController.leftTrigger().onTrue(new Commands.runOnce(()-> {System.out.println("I've Been Poked");
    //                                                               m_exampleSubsystem.debugMotor();}, m_exampleSubsystem));
    // m_driverController.leftTrigger().onFalse(new Commands.runOnce(()-> {m_exampleSubsystem.stop();}, m_exampleSubsystem));
    m_driverController.rightBumper().onTrue(m_exampleSubsystem.ChangePIDC());
    m_driverController.leftBumper().onTrue(m_exampleSubsystem.specificChangeSpeedCommand());

  }

  
}
