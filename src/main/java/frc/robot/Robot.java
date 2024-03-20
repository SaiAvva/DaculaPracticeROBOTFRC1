// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;



public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  //motors

  private TalonFX FrontRightMotor = new TalonFX(0);
  private TalonFX FrontLeftMotor = new TalonFX(1);
  private TalonFX BackRightMotor = new TalonFX(2);
  private TalonFX BackLeftMotor = new TalonFX(3);

  //joysticks
  private XboxController joystick = new XboxController(0);
  //conversion
  private final double Convert = 1.0 / 128 * 6 * Math.PI / 12;
  //encoders
  private Encoder encoder = new Encoder(0,1,false, EncodingType.k4X);
  //setpoint makes sure that robot doesnt forget command after use
  private double setpoint = 0;
  //currentPosition
  private double SensorPosition = encoder.get() * Convert;
  //kP
  private double kP = 0.43;


  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();
    
   
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  encoder.reset();
  }

  @Override
  public void autonomousPeriodic() {

    double error = setpoint - SensorPosition;
    double MotorOutput = kP * error;


    if(joystick.getRawButton(0)){
      setpoint = 10;

    }

    else if(joystick.getRawButton(1)){
      setpoint = 0;
    }

    while(setpoint == 10){
    setpoint = 0;

    }

    //MotorOutputs
    BackLeftMotor.set(MotorOutput);
    BackRightMotor.set(MotorOutput);
    FrontLeftMotor.set(MotorOutput);
    FrontRightMotor.set(MotorOutput);


  }

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}
}
