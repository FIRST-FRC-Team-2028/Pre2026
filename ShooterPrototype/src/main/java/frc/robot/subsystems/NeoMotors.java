// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.PrintStream;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.SendableSparkMax;
import frc.robot.SendableSparkPID;
import frc.robot.Constants.CanIDS;
import frc.robot.SendableRelEncoder;

public class NeoMotors extends SubsystemBase {

  // SparkMax neoMotorOne, neoMotortwo;
  SendableSparkMax neoMotorOne, neoMotortwo;
  SparkMaxConfig neoOneConfig;
  SparkMaxConfig neoTwoConfig;
  SparkClosedLoopController neoOneController;
  RelativeEncoder neoOneEncoder, neoTwoEncoder;
  SendableSparkPID sendableNeoOneController;
  SendableRelEncoder sendableNeoOneEncoder;

  double speed = 200.; //Neo 1650 Motor Max Speed 5676RPM
  double MotorIncrement = 200.;

  double Kp = 0.0002; 
  double Ki = 0.00000015; 
  double Kd = 0.015; 
  
  double Kpc,Kic,Kdc;

  /** Rotate Two Neo Motors In Opposite Directions. */ 
  public NeoMotors() {
    System.out.println("I have been executed");
    //neoMotorOne = new SparkMax(CanIDS.leaderMotor, MotorType.kBrushless); //Leader 
    //neoMotortwo = new SparkMax(CanIDS.followerMotor, MotorType.kBrushless); //Follower
    neoMotorOne = new SendableSparkMax(CanIDS.leaderMotor, MotorType.kBrushless); //Leader 
    neoMotortwo = new SendableSparkMax(CanIDS.followerMotor, MotorType.kBrushless); //Follower

    // Configs
    neoOneConfig = new SparkMaxConfig();
    neoOneConfig.idleMode(IdleMode.kCoast);
    neoTwoConfig = new SparkMaxConfig();
    neoTwoConfig.apply(neoOneConfig);
    neoOneConfig.closedLoop.pid(Kp, Ki, Kd);
    neoTwoConfig.follow(CanIDS.leaderMotor,true); 
    neoOneConfig.inverted(true);

    //Neo Motor Config
    neoMotorOne.configure(neoOneConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    neoMotortwo.configure(neoTwoConfig,ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    //Controller
    neoOneController = neoMotorOne.getClosedLoopController();
    sendableNeoOneController = new SendableSparkPID(neoMotorOne, Kp, Ki, Kd);

    neoOneEncoder = neoMotorOne.getEncoder();
    neoTwoEncoder = neoMotortwo.getEncoder();
    sendableNeoOneEncoder = new SendableRelEncoder(neoOneEncoder);

    //SmartDashBoard
    SmartDashboard.putNumber("Kp", Kp);
    SmartDashboard.putNumber("Ki", Ki);
    SmartDashboard.putNumber("Kd", Kd);
    addChild("Motor", neoMotorOne);
    addChild("Encoder", sendableNeoOneEncoder);
    addChild("PID", sendableNeoOneController);
    SmartDashboard.putData("ShooterMotor",neoMotorOne);
    SmartDashboard.putData("ShooterPID",sendableNeoOneController);
    SmartDashboard.putData("ShooterRPM",sendableNeoOneEncoder);
    SmartDashboard.putNumber("CommandSpeed", speed);
  }

  public void stop() {
    neoMotorOne.stopMotor();
  }
  
  public void run() {
    neoOneController.setSetpoint(speed, ControlType.kVelocity);
  }
  /** Multiplies the motor increments by [inc]. [inc] is changed by one when either changespeedP or changespeedN run.  */
  public double changeSpeed (int inc) {
    speed += inc * MotorIncrement;
    SmartDashboard.putNumber("CommandSpeed", speed);
    run();
    //System.out.println("Ran");
    return speed;
  }
  /** Adds the decided speed from [SpecificChangeSpeed] to the current speed */
  public double changeSpeed (double inc) {
    speed += inc;
    SmartDashboard.putNumber("CommandSpeed", speed);
    run();
    //System.out.println("Ran");
    return speed;
  }



  /** Changes speed based off decided amount. */
  public void SpecificChangeSpeed () {
    changeSpeed(SmartDashboard.getNumber("CommandSpeed", speed) - speed); 
  }

  public void debugMotor() {
    neoMotorOne.set(0.2);
    System.out.println("Debug Motor");
  }

  public void changePID() {
    Kpc = SmartDashboard.getNumber("Kp", 0.);
    Kic = SmartDashboard.getNumber("Ki", Ki);
    Kdc = SmartDashboard.getNumber("Kd", Kd);
    SmartDashboard.updateValues();
    neoOneConfig.closedLoop.pid(Kpc, Kic, Kdc);
    neoMotorOne.configure(neoOneConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    System.out.println(String.format("kp, ki,kd = %e,%e,%e", Kpc,Kic,Kdc));
    //System.out.println("PID Changed");
  }

  public Command closedLoopController() {
    return runOnce(()-> {run();});
  } 
   public Command stopC() {
    return runOnce(()-> {stop();});
  } 
   public Command changeSpeedP() {
    return runOnce(()-> {changeSpeed(+1);});
  } 
   public Command changeSpeedN() {
    return runOnce(()-> {changeSpeed(-1);});
  } 
   public Command specificChangeSpeedCommand() {
    return runOnce(()-> {SpecificChangeSpeed();});
  } 
   public Command ChangePIDC() {
    return runOnce(()-> {changePID();});
  }

  public double getp() {
    return neoMotorOne.configAccessor.closedLoop.getP();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Speed1", neoOneEncoder.getVelocity());
    SmartDashboard.putNumber("Speed2", neoTwoEncoder.getVelocity());
    SmartDashboard.putNumber("V1", neoMotorOne.getAppliedOutput());
    SmartDashboard.putNumber("V2", neoMotortwo.getAppliedOutput());
    SmartDashboard.putNumber("PidP", getp());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
