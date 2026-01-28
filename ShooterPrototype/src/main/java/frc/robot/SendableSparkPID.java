// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;

/** Make the PID controller for a SparkMax accessible on LiveWindow */
public class SendableSparkPID 
   //extends SparkClosedLoopController
   implements Sendable
{
    SparkClosedLoopController controller;
    SparkMaxConfig mConfig;
    SendableSparkMax motor;
    double target;
    ControlType controlType = ControlType.kVelocity;
    double kp, ki, kd;

    /** Builds a sendable controller for sparkmax motor */
    public SendableSparkPID(SendableSparkMax motor){
    this.motor = motor;
    mConfig = new SparkMaxConfig();
    controller = motor.getClosedLoopController();
   }

   public SendableSparkPID(SendableSparkMax motor, double p, double i, double d) {
    this(motor);
    kp = p;
    ki = i;
    kd = d;
    
   }

   void p(double val){
    kp = val;
    mConfig.closedLoop.p(val);
    motor.configure(mConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    System.out.println("I changes p");
    
   }
   void i(double val){
    ki = val;
    mConfig.closedLoop.i(val);
     motor.configure(mConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
   }
   void d(double val){
    kd = val;
    mConfig.closedLoop.d(val);
     motor.configure(mConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
   }
   void enable(boolean en){
    if(en)controller.setSetpoint(target, controlType);
    else motor.stopMotor();
   }
   void vel_Pos(boolean vel){
    if(vel)controlType = ControlType.kVelocity;
    else controlType = ControlType.kPosition;
   }
   void sp(double val){     // this method does not ensure val is a valid set point
    target = val;
   }

   public double getP() {
    return kp;
   }
   

   public double getI() {
    return ki;
   }

   public double getD() {
    return kd;
   }
   

   @Override
   public void initSendable(SendableBuilder builder){
       builder.setSmartDashboardType("PIDController");
       builder.addDoubleProperty("P", this::getP, this::p);
       builder.addDoubleProperty("I", this::getI, this::i);
       builder.addDoubleProperty("D", this::getD, this::d);
       builder.addDoubleProperty("SetPoint", null, this::sp);
       builder.addBooleanProperty("Enable", null, this::enable);
       builder.addBooleanProperty("Velocity", null, this::vel_Pos);
   }
}
