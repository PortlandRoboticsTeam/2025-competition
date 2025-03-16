// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.ArmPosition;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;
import frc.robot.subsystems.Motor.MotorType;
import frc.robot.subsystems.Encoder.EncoderType;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int driverControllerPort = 1;
    public static final int helperControllerPort = 0;

    public static final String AutonomousCommandName = "Copy of vision 2";
  }

  public static final double DEADBAND = 0.1;
  public static final double maximumSpeed = Units.feetToMeters(4.5);
  public static final TelemetryVerbosity telemetryVerbosity = TelemetryVerbosity.LOW;
    
    public static class GrabberConstants {
      public static final int coralMotorID = 18;
      public static final int algaeMotorID = 19;
      public static final double coralSpeed = .2;
      public static final double algaeSpeed = 0.35;
      public static final MotorType algaeMotorType = MotorType.Talon;
      public static final MotorType coralMotorType = MotorType.Talon;
      public static final double sencerdelay = 0.2;
    }
    public static class ArmConstants {
      public static final double manualControlJoystickDeaband = 0.1;
      public static final boolean useBounds = false;
  
      public static final double shoulderOffset = 27-60, shoulderMin =  0, shoulderMax = 184;
      public static final int    shoulder1ID    = 13, shoulder2ID = 14;
      public static final int shoulderEncoderID  = 0;
      public static final MotorType shoulderType = MotorType.Talon;
      public static final EncoderType shoulderEncoderType = EncoderType.DutyCycle;
  
  
      public static final double wristOffset = 0, wristMin =  0, wristMax = 0;
      public static final int wristID  = 16;
      public static final int wristEncoderID  = 17;
      public static final MotorType wristType = MotorType.Talon;
      public static final EncoderType wristEncoderType = EncoderType.CANCoder;
  
  
  
      public static final double telescopeOffset = 0, telescopeMin =  0, telescopeMax = 744;
      public static final int telescopeID  = 15;
      public static final int telescopeEncoderID  = 19;
      public static final int greenThreshold = 40000;
      public static final MotorType telescopeType = MotorType.Talon;
      public static final EncoderType telescopeEncoderType = EncoderType.CANCoder;
      public static final double telescopeCalibrationSpeed = -.2;
  
      public static final ArmPosition positionCommandCompletionTolerance = new ArmPosition(5, .2, 1, "tolerance (not a position)");
  
  
      public static final ArmPosition[] positions = {
        //min 32
        new ArmPosition(54.42, 0.2,240, "Rest: 0"),
  
        new ArmPosition(37, .55, 324,     "Tray: 1"),  //  3/10/25
        new ArmPosition(36, .08, 276,  "Low Foward Coral: 2"),  //  3/10/25
        new ArmPosition(33.3, 1.9, 202,"Low Back Coral: 3"),
        new ArmPosition(45, -.17, 32.7,   "Low Reef Ball: 4"),
        new ArmPosition(60, 1.88, 291,   "Mid Foward Coral: 5"),  //  3/10/25
        new ArmPosition(56, 2.1, 199,  "Mid Back Coral: 6"),
        new ArmPosition(56.18, 1.36, 10, "High Reef Ball: 7"),
        new ArmPosition(77.22, 8.87, -10, "High Foward Coral: 8"),  //  3/10/25
        new ArmPosition(78, 6.16, 224, "High Back Coral: 9"),
  
        new ArmPosition(93, .11, 35, "Ready to Climb: 10"),
        new ArmPosition(2.7, 1.88, 50, "Climbing: 11"),
        new ArmPosition(99, 4, 0, "Depricated: 12"),
        new ArmPosition(99, 4, 0, "Depricated: 13"),
        new ArmPosition(99, 4, 0, "Depricated: 14"),
        new ArmPosition(99, 4, 0, "Depricated: 15"),
  
        new ArmPosition(56, .89, 9.5, "Collect Coral Human: 16"),//145,-.34,274
        new ArmPosition(99, 4, 0, "Depricated: 17"),
        new ArmPosition(99, 4, 0, "Depricated: 18"),
  
        new ArmPosition(80, 0, 324, "Intermediate Position: 19"),
        new ArmPosition(77.22, 8.87, 324, "Intermediate High: 20"),  //  3/10/25
    };    
  }
}
