// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Power;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveModuleSubsystem extends SubsystemBase {
  private TalonFX motorAngle;
  private TalonFX motorDrive;
  private PIDController pid;
  private double tolerance = 1.5;
  private double wantedAngle;
  
  /** Creates a new SwerveModuleSubsystem. */
  public SwerveModuleSubsystem() {
    motorAngle = new TalonFX(8, "swerve");
    motorDrive = new TalonFX(7, "swerve");
    pid = new PIDController(0.01, 0, 0);
    pid.setTolerance(tolerance);
    SmartDashboard.putNumber("kp", 0.1);
    
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    SmartDashboard.putNumber("drive speed", this.getSpeed().getRotations());
    SmartDashboard.putNumber("angle", this.getAngle().getDegrees());
    SmartDashboard.putNumber("angle power", motorAngle.get());
    SmartDashboard.putNumber("drive power", motorDrive.get());
    double kp = SmartDashboard.getNumber("kp", 0.1);
    pid.setP(kp);
   

  }

  public Command goToState(double wontedSpeed, double wantedAngle){
    this.wantedAngle = wantedAngle;
    return Commands.runEnd( () -> {
      double power = pid.calculate(this.getAngle().getDegrees() , wantedAngle);
      this.setPowerDrive(wontedSpeed);
      this.setPowerAngle(power);
    
    }, () -> {
      this.stop();
    }, this);
  }

  public void setPowerAngle(double power){
    motorAngle.set(power);
  }

  public void stopAngle(){
    motorAngle.set(0);
  }

  public Rotation2d getAngle(){
    Rotation2d samthing = Rotation2d.fromRotations(motorAngle.getPosition().getValueAsDouble());
    return samthing;
  }

  public void stopDrive(){
    motorDrive.set(0);
  }

  public void stop(){
    motorAngle.set(0);
    motorDrive.set(0);
  }

  public void setPowerDrive(double power){
    motorDrive.set(power);
  
  }

  public Rotation2d getSpeed(){
    Rotation2d samthing = Rotation2d.fromRotations(motorDrive.getVelocity().getValueAsDouble());
    return samthing;
  }

  public Command rotate(Rotation2d rotation){
    return Commands.run( () -> {
      if (this.getAngle().getDegrees() > rotation.getDegrees()) {
        this.setPowerAngle(-0.3);
      }
      else {
        this.setPowerAngle(0.3);
      }
    }, this).until( () -> {
      return this.getAngle().getDegrees() > rotation.getDegrees() - 1 && this.getAngle().getDegrees() < rotation.getDegrees() + 1;
    }).finallyDo( () -> {
      this.stop();
    });
  }

  public boolean atAngle(){
    return pid.atSetpoint();
  }
  


}
