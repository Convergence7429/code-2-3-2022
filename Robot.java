// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;

import javax.sound.sampled.SourceDataLine;

//import com.ctre.phoenix.motorcontrol.ControlMode;
//import com.ctre.phoenix.motorcontrol.can.*;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
//import edu.wpi.first.wpilibj.
//import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAlternateEncoder.Type;

import edu.wpi.first.wpilibj.Timer;

import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.util.WPILibVersion;

public class Robot extends TimedRobot {

  Timer autoTimer = new Timer();

  public Drivetrain drive = new Drivetrain();

  int count = 0;

  static Joystick xbox = new Joystick(1);

  double positionGoalStayInPlace = 0.0;

  int stage = 1;

  NetworkTable limelight;
  Limelight forwardCamera;
  
  @Override
  public void robotInit() {
    
    drive.init();
    //initialize networktables instance, grab the limelight table
    NetworkTableInstance table = NetworkTableInstance.getDefault();
    limelight = table.getTable("limelight");
    
    
  }

  @Override
  public void autonomousInit() {
    drive.init();
    autoTimer.reset();
    autoTimer.start();
    stage = 1;
    //initialize the camera withe the table
    forwardCamera = new Limelight(limelight);
  }

  @Override
  public void autonomousPeriodic() {
    System.out.println("stage: " + stage);
    System.out.println("timer: " + autoTimer.get() + " seconds");
    System.out.println("fl motor: " + drive.flMotor.getEncoder().getPosition());

      switch(stage) {
      case 1 : {
        drive.driveTrainByInches(50.0, 0);
        if(autoTimer.get() > 4.0){
          drive.resetDriveTrainEncoders();
          stage = 2;
        }
        break;
      }
      case 2 : {
        drive.driveTrainByInches(100.0, 1);
        if(autoTimer.get() > 15.0){
          drive.resetDriveTrainEncoders();
          stage = 3;
        }
        break;
      }
      case 3 : {
        drive.driveTrainByInches(50.0, 0);
        if(autoTimer.get() > 20.0){
          drive.resetDriveTrainEncoders();
          stage = 4;
        }
        break;
      }
  }
}
    



    // if(forwardCamera.visible())
    // {
    //   System.out.println("angleOffset: " + forwardCamera.angleOffsetX());

    //   double speed = strafeController.quadraticPositionAndSpeed(forwardCamera.angleOffsetX(), forwardCamera.angleOffsetX());
      
    //   //strafe with the calculated speed towards the ball
    //   if(forwardCamera.angleOffsetX() < -3.0){
    //     drive.mecanumDrive.driveCartesian(0,speed,0);
    //   } else if(forwardCamera.angleOffsetX() > 3.0) {
    //     drive.mecanumDrive.driveCartesian(0,-speed,0);
    //   }
      
    //   System.out.println("I see the target");
    // } else{
    //   System.out.println("I do not see the target");
    // }
  

  @Override
  public void teleopInit() {
    //drive.init();
    //drive.resetDriveTrainEncoders();
    drive.turningTimer.start();
    count = 0;
  }

  @Override
  public void teleopPeriodic() {
    // if(count == 0){
    //   positionGoalStayInPlace = drive.flMotor.getEncoder();
    // }
    //drive.drivetrainTeleop();
    System.out.println("position: " + drive.flMotor.getEncoder().getPosition());
    //System.out.println("speed: " + drive.flMotor.get());
    //System.out.println(drive.flMotor.getAppliedOutput());
    //System.out.println("output current: " + drive.flMotor.getOutputCurrent());
    //drive.flMotor.set(0.05);
    //drive.flMotor.set(-0.2);
    
    // double speed = 0.0;
    // if(drive.flMotor.getEncoder().getPosition() > 100){
    //   speed = drive.maxMotorSpeedEncoders;
    // } else {
    //   if(drive.flMotor.getEncoder().getPosition() > 0){
    //     speed = Math.pow((drive.flMotor.getEncoder().getPosition() / (100 * (1.0/drive.maxMotorSpeedEncoders))), 2);
    //   } else {
    //     speed = Math.pow((drive.flMotor.getEncoder().getPosition() / (100 * (1.0/drive.maxMotorSpeedEncoders))), 2) * -1.0;
    //   }
    // }
    // drive.flMotor.set(-speed);

    if(drive.flMotor.getEncoder().getPosition() > 2.0){
      //positionGoalStayInPlace = drive.flMotor.getEncoder().getPosition();
      //drive.flMotor.set(Robot.quadraticPositionAndSpeed(-0.05, -0.4, drive.flMotor.getEncoder().getPosition(), drive.flMotor.getEncoder().getPosition()));
      drive.flMotor.set(-0.05);
    } else if (drive.flMotor.getEncoder().getPosition() < -2.0){
      drive.flMotor.set(0.05);
    } else {
      drive.flMotor.set(0.0);
    }


    // if(drive.flMotor.getEncoder().getPosition() < -10){
    //   //drive.flMotor.set(Robot.quadraticPositionAndSpeed(0.05, 0.4, drive.flMotor.getEncoder().getPosition(), drive.flMotor.getEncoder().getPosition()));
    //   drive.flMotor.set(0.05);
    // } else {
    //   drive.flMotor.set(0);
    // }

    //drive.flMotor.set(Robot.quadraticPositionAndSpeed(0.1, 0.4, , currentPosition))




  }
    
  

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {}

  public static double quadraticPositionAndSpeed(double minimumMotorSpeed, double maximumMotorSpeed, double positionGoal, double currentPosition) {

    // position could be angle offset, encoder count, inches, etc.

    // maximum speed should be reached at the middle position

    // need to look at system of equations again and see if I want endpoint to just be positionGoal with speed 0. (position, 0)

    double a = ((positionGoal * maximumMotorSpeed - minimumMotorSpeed * positionGoal - minimumMotorSpeed * (positionGoal / 2) + minimumMotorSpeed * (positionGoal / 2)) / (positionGoal * (positionGoal / 2) * ((positionGoal / 2) - positionGoal)));

    double b = ((maximumMotorSpeed - a * (positionGoal / 2) * (positionGoal / 2) - minimumMotorSpeed) / (positionGoal / 2));

    double speed = a * currentPosition * currentPosition + b * currentPosition + minimumMotorSpeed;
    
    return speed; // value would be from 0.0 to 1.0 like any motor needs to be
  }

}

