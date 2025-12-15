// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;

import org.json.simple.parser.ParseException;

import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveModule.DriveRequestType;
/* import com.fasterxml.jackson.databind.util.Named;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.events.EventTrigger;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import com.pathplanner.lib.util.FileVersionException; */
import frc.robot.Constants.OIConstants;
//import frc.robot.Constants.PathPlannerConstants;
//import frc.robot.commands.RunWheels;
import frc.robot.commands.DriveCommand;
//import frc.robot.commands.DriveToAprilTagPP;
import frc.robot.commands.DriveToReefTag;
//import frc.robot.commands.ElevatorPosition;
//import frc.robot.commands.HandlerPosition;
import frc.robot.commands.MeasuredDrive;
//import frc.robot.commands.SpitSequence; // Stuff using is Commented out
import frc.robot.commands.TimedSpeedDrive;
import frc.robot.commands.TurnToReef;
import frc.robot.subsystems.AprilCamera;
import frc.robot.subsystems.Drivetrain;
//import frc.robot.subsystems.Elevator;
//import frc.robot.subsystems.Handler;
//import frc.robot.subsystems.Lights;
import frc.robot.subsystems.MatchTimer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj.Filesystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final Drivetrain driveSubsystem;
  //private final Elevator elevatorSubsystem;
  //private final Handler handlerSubsystem;
  private final AprilCamera april;
  //private final Lights lights;
  private final MatchTimer matchTimer;
  private final SendableChooser<Command> autoChooser;
  public boolean algae = false;
  BufferedWriter testResults;
    

  // Joysticks
    private final Joystick driverJoytick = new Joystick(OIConstants.kDriverControllerPort);
    //private final Joystick mechJoytick1 = new Joystick(OIConstants.kMechControllerPort);
    //private final Joystick mechJoytick2 = new Joystick(OIConstants.kMechControllerPort2);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  @SuppressWarnings("unused")
  public RobotContainer() {
    matchTimer = new MatchTimer();
    if (Constants.DRIVE_AVAILABLE){
      driveSubsystem = new Drivetrain(this);
    } else driveSubsystem = null;
    if (Constants.CAMERA_AVAILABLE){
      april = new AprilCamera(this);
    } else april = null;

    if (Constants.DRIVE_AVAILABLE){
      driveSubsystem.setDefaultCommand(new DriveCommand(driveSubsystem));
      
    } 
    autoChooser=null;
    if (DriverStation.getAlliance().get() == DriverStation.Alliance.Red){
      
    };
    
    try {
       //File root = Filesystem.getOperatingDirectory().
       testResults = new BufferedWriter(new FileWriter("experiment.csv"));
       testResults.write("height, pitch, x, y, heading, output yaw, pitch, area, x, y, heading, dist");
       testResults.flush();
    } catch (IOException e) {
      System.out.print("Write experiment results where? Not");
    }
    // Configure the trigger bindings
    configureButtonBindings();
  }

  double[] dummy = {-999.,-999.,-999.};
  private void printExper() {
   try {

    testResults.write(
      String.format("%5.3f ,",SmartDashboard.getNumber("cameraHeight",-999.))
      +String.format("%5.3f ,",SmartDashboard.getNumber("cameraPitch",-999.))
      +String.format("%5.3f ,",SmartDashboard.getNumber("cameraX",-999.))
      +String.format("%5.3f ,",SmartDashboard.getNumber("cameraY",-999.))
      +String.format("%5.3f ,",SmartDashboard.getNumber("cameraHeading",-999.))
      +String.format("%5.3f ,",SmartDashboard.getNumber("targetYaw",-999.))
      +String.format("%5.3f ,",SmartDashboard.getNumber("targetPitch",-999.))
      +String.format("%5.3f ,",SmartDashboard.getNumber("targetArea",-999.))
      +String.format("%5.3f ,",SmartDashboard.getNumberArray("rawT3d",dummy)[0])
      +String.format("%5.3f ,",SmartDashboard.getNumberArray("rawT3d",dummy)[1])
      +String.format("%5.3f ,",SmartDashboard.getNumberArray("rawT3d",dummy)[2])
      +String.format("%5.3f",SmartDashboard.getNumber("targetDistanceInches",-999.))
    );
   }catch (IOException e) {
    System.out.println("Bad write of experiment data!");
   }
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
  @SuppressWarnings("unused")
  public void configureButtonBindings() {
    CommandScheduler.getInstance().getActiveButtonLoop().clear();
    
    new JoystickButton(driverJoytick, OIConstants.STORE_RESULTS)
        .onTrue(new InstantCommand(() -> printExper()
             ));
    

    if (Constants.DRIVE_AVAILABLE) {
      //new JoystickButton(driverJoytick, 3)
      //  .onTrue(new TimedDrive(driveSubsystem, 0.25, 0, 0.5, 0));
      /*new JoystickButton(driverJoytick, 2)
        .onTrue(new TimedSpeedDrive(driveSubsystem, 0.25, 0, -0.5, 0));*/
      //reset gyro
      new JoystickButton(driverJoytick, OIConstants.kResetGyro)
        .onTrue(new InstantCommand(() -> driveSubsystem.resetGyro()));
      //new JoystickButton(driverJoytick, 3)
      //  .whileTrue(new DriveToAprilTagPP(driveSubsystem, april));
      /*new JoystickButton(driverJoytick, 2)
        .whileTrue(new MeasuredDrive(driveSubsystem, 0, -6.5));
      new JoystickButton(driverJoytick, 3)
        .whileTrue(new MeasuredDrive(driveSubsystem, 0, 6.5));
      //new JoystickButton(driverJoytick, 3)*/
      if (Constants.CAMERA_AVAILABLE){
        //  .whileTrue(new DriveToReefTag(driveSubsystem, april));    
      new JoystickButton(driverJoytick, 4)
        .whileTrue(new DriveToReefTag(driveSubsystem, april)
        .andThen(new InstantCommand(() -> SmartDashboard.putBoolean("Drive Finished", true))));
      
      new JoystickButton(driverJoytick, 4)
        .onFalse(new InstantCommand(() -> SmartDashboard.putBoolean("Drive Finished", false))
        .andThen(new MeasuredDrive(driveSubsystem, 20., 0)));
      
      /*new JoystickButton(driverJoytick, 2)
        .whileTrue(new DriveToReefTag(driveSubsystem, april)
        .andThen(new MeasuredDrive(driveSubsystem, 0, -6.5)));
      new JoystickButton(driverJoytick, 3)
        .whileTrue(new DriveToReefTag(driveSubsystem, april)
        .andThen(new MeasuredDrive(driveSubsystem, 0, 6.5))); */

              
      new JoystickButton(driverJoytick, 2)
        .whileTrue(new DriveToReefTag(driveSubsystem, april)
        //.andThen(new ElevatorPosition(elevatorSubsystem, ElevatorConstants.L3)
        .andThen(new MeasuredDrive(driveSubsystem, 0., -6.75))); //11.5 //-6.5
        /*.andThen(new WaitCommand(.5))
        .andThen(new RunWheels(handlerSubsystem, HandlerConstants.outputSpeed, 1, false)));*/
      /*new JoystickButton(driverJoytick, 3)
        .whileTrue(new DriveToReefTag(driveSubsystem, april)
        .andThen(new MeasuredDrive(driveSubsystem, 0, 6.5))
        .andThen(new ElevatorPosition(elevatorSubsystem, ElevatorConstants.L2)));*/

      new JoystickButton(driverJoytick, 3)
        .whileTrue(new DriveToReefTag(driveSubsystem, april)
        .andThen(new MeasuredDrive(driveSubsystem, 0., 6.75))); //11.5 
        //.andThen(new ElevatorPosition(elevatorSubsystem, ElevatorConstants.L3)));
       /* .andThen(new WaitCommand(.5))
        .andThen(new RunWheels(handlerSubsystem, HandlerConstants.outputSpeed, 1, false)));*/
      }
      
      /*//Pathplanner drive to blue, right coral station
      new JoystickButton(driverJoytick, OIConstants.kRightCoralStation).and(()->DriverStation.getAlliance().get() == DriverStation.Alliance.Blue)
        .whileTrue(driveSubsystem.pathfindToPose(PathPlannerConstants.blueRightStationX, PathPlannerConstants.blueRightStationY, 
                                                 PathPlannerConstants.blueRightStationRot, PathPlannerConstants.coralStationEndVelocity));
      //Pathplanner drive to blue, left coral station
      new JoystickButton(driverJoytick, OIConstants.kLeftCoralStation).and(()->DriverStation.getAlliance().get() == DriverStation.Alliance.Blue)
        .whileTrue(driveSubsystem.pathfindToPose(PathPlannerConstants.blueLeftStationX, PathPlannerConstants.blueLeftStationY, 
                                                 PathPlannerConstants.blueLeftStationRot, PathPlannerConstants.coralStationEndVelocity));
      //Pathplanner drive to red, right coral station
      new JoystickButton(driverJoytick, OIConstants.kRightCoralStation).and(()->DriverStation.getAlliance().get() == DriverStation.Alliance.Red)
        .whileTrue(driveSubsystem.pathfindToPose(PathPlannerConstants.redRightStationX, PathPlannerConstants.redRightStationY, 
                                                 PathPlannerConstants.redRightStationRot, PathPlannerConstants.coralStationEndVelocity));
      //Pathplanner drive to red, left coral station                     
      new JoystickButton(driverJoytick, OIConstants.kLeftCoralStation).and(()->DriverStation.getAlliance().get() == DriverStation.Alliance.Red)
        .whileTrue(driveSubsystem.pathfindToPose(PathPlannerConstants.blueLeftStationX, PathPlannerConstants.blueLeftStationY, 
                                                 PathPlannerConstants.blueLeftStationRot, PathPlannerConstants.coralStationEndVelocity));
      //Pathplanner drive to blue barge
      new JoystickButton(driverJoytick, OIConstants.kDriveToBarge).and(()->DriverStation.getAlliance().get() == DriverStation.Alliance.Blue)
        .whileTrue(driveSubsystem.pathfindToPose(PathPlannerConstants.blueBargeX, PathPlannerConstants.blueBargeY, 
                                                 PathPlannerConstants.blueBargeRot, PathPlannerConstants.bargeEndVelocity));
      //Pathplanner drive to red barge
      new JoystickButton(driverJoytick, OIConstants.kDriveToBarge).and(()->DriverStation.getAlliance().get() == DriverStation.Alliance.Red)
        .whileTrue(driveSubsystem.pathfindToPose(PathPlannerConstants.redBargeX, PathPlannerConstants.redBargeY, 
                                                 PathPlannerConstants.redBargeRot, PathPlannerConstants.bargeEndVelocity));*/
      //Drive robot back enough to score on L4
      /*new JoystickButton(driverJoytick, OIConstants.kBackUp)
      .onTrue(new TimedDrive(driveSubsystem, 0.25, -0.5, 0, 0));*/
      

      /*new JoystickButton(driverJoytick, OIConstants.kpathfindTopCoralStation)
        .onTrue( driveSubsystem.pathfindToPath("Top Coral Station"));*/
      //new JoystickButton(driverJoytick, 2)
      //  .onTrue(driveSubsystem.pathfindToPose(1.25, 1, 0, 0));
    }


      
      // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.


  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return autoChooser.getSelected();
  }
  public void endRobotCode() {
    double end;
    end = 10/0;
  }

  public Drivetrain getDrivetrain(){
    return driveSubsystem;
  }


  public AprilCamera getApril(){
    return april;
  }

  
  public MatchTimer getMatchTimer(){
    return matchTimer;
  }
}
