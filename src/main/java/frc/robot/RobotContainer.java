// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;

import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix6.hardware.CANcoder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import frc.robot.commands.PosidonDrive;
import frc.robot.commands.drive;

import frc.robot.commands.setModualState;
import frc.robot.subsystems.PosidenModual;
import frc.robot.subsystems.PosidonSwerve;
import frc.robot.subsystems.SwerveSubsystem;



public class RobotContainer {
  PosidonSwerve swerve = new PosidonSwerve();
  //PosidenModual moduleTest = new PosidenModual(0, 1, 2, 3, 0);
  

  // SwerveSubsystem swerve = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),"swerve"));
  CommandGenericHID controler = new CommandGenericHID(0);

  public RobotContainer() {
    // swerve.setDefaultCommand(new drive(swerve, ()->0, ()->0, ()->0, ()->false, ()->false, ()->false, ()->false));
    swerve.setDefaultCommand(new PosidonDrive(swerve,
    ()->MathUtil.applyDeadband( controler.getRawAxis(1),Constants.LEFT_Y_DEADBAND),
    ()->MathUtil.applyDeadband( controler.getRawAxis(0),Constants.RIGHT_X_DEADBAND),
    ()->MathUtil.applyDeadband(controler.getRawAxis(2), Constants.DEADBAND),
    false));
    //moduleTest.setDefaultCommand(new setModualState(moduleTest, ()->-MathUtil.applyDeadband( controler.getRawAxis(1),Constants.RIGHT_X_DEADBAND),()->MathUtil.applyDeadband( controler.getRawAxis(0),Constants.RIGHT_X_DEADBAND)));
    configureBindings();
  }

  private void configureBindings() {}

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
