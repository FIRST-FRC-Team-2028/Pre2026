// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.ResetMode;
import com.revrobotics.PersistMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.SendableRelEncoder;
import frc.robot.SendableSparkMax;
import frc.robot.SendableSparkPID;

public class ExampleSubsystem extends SubsystemBase {
  //SparkMax motor;
  SendableSparkMax motor;
  SparkBaseConfig mConfig;
  RelativeEncoder encoder;
  SendableRelEncoder msencoder;
  EncoderConfig eConfig;
  SparkClosedLoopController clc;
  SendableSparkPID msclc;
  double clcTarget;
  ClosedLoopConfig clcc;
  /** Creates a new ExampleSubsystem. */
  public ExampleSubsystem() {
    motor = new SendableSparkMax(61, SparkLowLevel.MotorType.kBrushless);
    mConfig = new SparkMaxConfig();
    mConfig.idleMode(IdleMode.kBrake);
    mConfig.openLoopRampRate(1.8); // seconds to go from 0 to full

    encoder = motor.getEncoder();
    eConfig = new EncoderConfig();
    eConfig.positionConversionFactor(Constants.Rollers.radius
                                    *Constants.Rollers.gearRatio);  // motor rotations : inches
    mConfig.apply(eConfig);

    clc = motor.getClosedLoopController();
    clcc = new ClosedLoopConfig();
    clcc.pid(1./10.,0.,0.);  // 10 inch offset generates full power
    mConfig.apply(clcc);
    /* Soft Limit */
    mConfig.softLimit.forwardSoftLimit(10.);
    mConfig.softLimit.forwardSoftLimitEnabled(true);

    motor.configure(mConfig, ResetMode.kResetSafeParameters
                           , PersistMode.kPersistParameters);
    
    // make encoder and controller Sendable
    msclc = new SendableSparkPID(motor);
    msencoder = new SendableRelEncoder(encoder);
    addChild("Motor", motor);
    addChild("Encoder", msencoder);
    addChild("Controller", msclc);
  }

  /** Move motor:
   * <p> Speed is :
   * <ul>
   *    <li>0.3 (with no argument)
   *    <li>+-0.3 depending on boolean argument
   *    <li>double speed as specified 
   * </ul>
   */
  public void setSpeed() {
    motor.set(0.3);
  }
  /** Move motor:
   * <p> Speed is :
   * <ul>
   *    <li>0.3 (with no argument)
   *    <li>+-0.3 depending on boolean argument
   *    <li>double speed as specified 
   * </ul>
   */
  public void setSpeed(boolean forward) {
    double direction = forward? 1: -1.;
    motor.set(0.3*direction);
  }
  /** Move motor:
   * <p> Speed is :
   * <ul>
   *    <li>0.3 (with no argument)
   *    <li>+-0.3 depending on boolean argument
   *    <li>double speed as specified 
   * </ul>
   */
  public void setSpeed(double speed) {
    motor.set(speed);
  }
  /** stop everything in this subsystem */
  public void stop() {
    setSpeed(0.);
  }

  /** set a start point for moving */
  public void zeroEncoder(){
    encoder.setPosition(0.);
  }

  /**
   * Simple command to zero the roller.
   *
   * @return a command
   */
  public Command exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          zeroEncoder();
        });
  }

  /** run motor to move 4 inches forward (open loop controller)*/
  public Command moveDist() {
    return runOnce(
        () -> {
          zeroEncoder();
          setSpeed();
        }).andThen(new WaitUntilCommand(()-> {
           return getPosition() > 4.;
        })).andThen(() -> {stop();});
  }/** run motor to move 4 inches forward or back */
  public Command moveDist(boolean forward) {
    return runOnce(
        () -> {
          zeroEncoder();
          setSpeed(forward);
        }).andThen(new WaitUntilCommand(()-> {
           return Math.abs(getPosition()) > 4.;
        })).andThen(() -> {stop();});
  }
  /** run motor to move specified distance */
  public Command moveDist(double dist) {
    return runOnce(
        () -> {
          zeroEncoder();
          setSpeed(dist>0.);
        }).andThen(new WaitUntilCommand(()-> {
           return Math.abs(getPosition()) > dist;
        })).andThen(() -> stop());
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }
  
  /** report motor position (inches) */
  public double getPosition() {
    return encoder.getPosition();
  }

  /** closed loop position control to target, inches */
  public void closedLoop(double target) {
    clcTarget = target;
    clc.setSetpoint(clcTarget, SparkBase.ControlType.kPosition);
  }

  /** closed loop position control to 4 inches */
  public void closedLoop() {
    closedLoop(4.);
  }

  /** run motor to move 4 inches forward (open loop controller)*/
  public Command clDist() {
    return runOnce(
        () -> {
          zeroEncoder();
          closedLoop();
        });
  }
  /** run motor to move specified distance */
  public Command clDist(double dist) {
    return runOnce(
        () -> {
          zeroEncoder();
          closedLoop(dist);
        });
  }
  /** run motor to move specified distance */
  public Command clDist(DoubleSupplier dist) {
    return runOnce(
        () -> {
          closedLoop(dist.getAsDouble()*5.);
        });
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Position", getPosition());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
