// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import frc.robot.Constants.OperatorConstants;
// import frc.robot.commands.ArmPositionCommandGenerator;
import frc.robot.commands.GenericCommand;
import frc.robot.commands.JointToPosition;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.ArmPosition;
import frc.robot.subsystems.Grabber;
import frc.robot.subsystems.Joint;
import frc.robot.subsystems.Motor;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.Telescope;
import frc.robot.subsystems.Motor.idleMode;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import static edu.wpi.first.units.Units.Rotation;

import com.pathplanner.lib.auto.NamedCommands;
// import com.studica.frc.AHRS;
// import com.studica.frc.AHRS.NavXComType;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  
  Motor climb = new Motor(31,Motor.MotorType.Talon);
  // hardware objects vv
  private final SwerveSubsystem swerve = new SwerveSubsystem();
  Joint shoulder1     = new Joint    (0, ArmConstants.shoulder2ID, ArmConstants.shoulderEncoderID , false, 0, 0.13, 0, 0.001, ArmConstants.shoulderType, ArmConstants.shoulderEncoderType);
  Joint wrist         = new Joint    (1, ArmConstants.wristID    , ArmConstants.wristEncoderID    , false, 0, 0.005, 0, 0, ArmConstants.wristType    , ArmConstants.wristEncoderType    );
  Telescope telescope = new Telescope(2, ArmConstants.telescopeID, ArmConstants.telescopeEncoderID, true , 0, 0.65, 0, 0, ArmConstants.telescopeType, ArmConstants.telescopeEncoderType);
  // ArmPositionCommandGenerator armCommandGenerator = new ArmPositionCommandGenerator(shoulder1, shoulder2, telescope, wrist);
  Grabber grabber = new Grabber();
  // PIDController steeringController = new PIDController(.03, 0, 0);
  // AHRS navx = new AHRS(NavXComType.kMXP_SPI);


  // controllers vv
  private final static CommandPS4Controller m_driverController =
        new CommandPS4Controller(OperatorConstants.driverControllerPort);
  private final static CommandPS4Controller m_helperController =
        new CommandPS4Controller(OperatorConstants.helperControllerPort);
  
  
  // variables vv
  private Command driveCommand; 
  // private double steeringTargetAngle = 0;
  private Command visdriveR = swerve.vishionDriveR();
  private Command visdriveL = swerve.vishionDriveL();
  private Command visdrivePickup = swerve.vishionDrivePickup();
  


  // commands vv
  Command resetGyro = swerve.getResetGyro();//.andThen(()->{steeringTargetAngle=0;});
  InstantCommand[] goToPositionCommand = new InstantCommand[ArmConstants.positions.length];
  InstantCommand grabCommand = new InstantCommand(()->grabber.grab());
  InstantCommand releaseCommand = new InstantCommand(()->grabber.release());
  InstantCommand stopGrabbingCommand = new InstantCommand(()->grabber.stop());
  GenericCommand runArmManualCommand = new GenericCommand(
    ()->{
      shoulder1.disablePID();
      // wrist    .disablePID();
      telescope.disablePID();
    },
    ()->{
      shoulder1.setSpeed(MathUtil.applyDeadband(-m_helperController.getLeftY (), ArmConstants.manualControlJoystickDeaband));
      // shoulder2.setSpeed(MathUtil.applyDeadband(-m_helperController.getLeftY (), ArmConstants.manualControlJoystickDeaband));
      // wrist    .setSpeed(MathUtil.applyDeadband(-m_helperController.getLeftX (), ArmConstants.manualControlJoystickDeaband)/5);
      telescope.setSpeed(MathUtil.applyDeadband(-m_helperController.getRightY(), ArmConstants.manualControlJoystickDeaband));
      // shoulder1.setSetpoint(shoulder1.getSetpoint() + MathUtil.applyDeadband(-m_helperController.getLeftY (), ArmConstants.manualControlJoystickDeaband));
      // shoulder2.setSetpoint(shoulder1.getSetpoint());
      wrist    .setSetpoint(wrist    .getSetpoint() + MathUtil.applyDeadband( m_helperController.getLeftX (), ArmConstants.manualControlJoystickDeaband));
      // telescope.setSetpoint(telescope.getSetpoint() - 0.1*MathUtil.applyDeadband( m_helperController.getRightY(), ArmConstants.manualControlJoystickDeaband));
    },
    ()->{
      shoulder1.setSetpoint(shoulder1.getAngleDegrees());
      // shoulder2.setSetpoint(shoulder1.getAngleDegrees());
      wrist    .setSetpoint(wrist    .getAngleDegrees());
      telescope.setSetpoint(telescope.getAngleDegrees());
      shoulder1.enablePID();
      shoulder1.enablePID();
      wrist    .enablePID();
      if(!telescope.isCalibrating()) telescope.enablePID();
    }
  );
  InstantCommand calibrateTelescope = new InstantCommand(()->{telescope.calibrateWithColors();});


    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {

      // defining the drive command
      driveCommand = swerve.driveCommand(
        () -> -MathUtil.applyDeadband(m_helperController.getL2Axis() > 0.5 ? 
                  m_helperController.getLeftY() : 
                  m_driverController.getRawAxis(1), Constants.DEADBAND),
        () -> -MathUtil.applyDeadband(m_helperController.getL2Axis() > 0.5 ? 
                  m_helperController.getLeftX() : 
                  m_driverController.getRawAxis(0), Constants.DEADBAND),
        () ->  MathUtil.applyDeadband(m_helperController.getL2Axis() > 0.5 ? 
                  m_helperController.getRightX() : 
                  m_driverController.getRawAxis(2), Constants.DEADBAND),
        () ->  false,//isFieldOriented
        () ->  -MathUtil.applyDeadband(m_driverController.getRawAxis(5),.4)
      );
      
      swerve.setDefaultCommand(driveCommand);

      configureArmSystems();
      registerCommands();
      configureBindings();
      
    }
    private void registerCommands(){
      NamedCommands.registerCommand("grab", grabCommand);
      NamedCommands.registerCommand("release", releaseCommand);
      NamedCommands.registerCommand("stopGrabbing", stopGrabbingCommand);
      NamedCommands.registerCommand("visdriveR", visdriveR);
      NamedCommands.registerCommand("visdriveL", visdriveL);
      NamedCommands.registerCommand("driveCommand", driveCommand);
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
      //ian temp bindings
      m_driverController.cross().onTrue(new InstantCommand(()->climb.set(1)));
      m_driverController.cross().onFalse(new InstantCommand(()->climb.set(0)));
       
      
      
      m_driverController.square().onTrue(resetGyro);
      m_driverController.R2().onTrue(grabCommand);
      m_driverController.L2().onTrue(releaseCommand);
      m_driverController.R1().onTrue(visdriveR);
      m_driverController.L1().onTrue(visdriveL);
      m_driverController.povUp().onTrue(visdrivePickup);
      
      m_driverController.R1().or(m_driverController.L1()).onFalse(driveCommand);

      CommandPS4Controller hc = m_helperController;
      Trigger noTriggersOrBumpers = hc.L1().or(hc.L2()).or(hc.R1()).or(hc.R2()).negate();

      Trigger rightBumperOnly = hc.L2().or(hc.L1()).or(hc.R2()).negate().and(hc.povCenter()).and(hc.R1());
      rightBumperOnly.and(hc.cross()).onTrue(goToPositionCommand[ 0]);
      rightBumperOnly.and(hc.square()).onTrue(resetGyro);
      hc.R2().and(hc.R1().and(hc.L1()).and(hc.L2()).negate()).whileTrue(runArmManualCommand);
      rightBumperOnly.and(hc.triangle()).onTrue(calibrateTelescope);
      
      noTriggersOrBumpers.and(hc.povDown  ()).and(hc.cross   ()).onTrue(goToPositionCommand[ 1]);// Tray: 1
      noTriggersOrBumpers.and(hc.povCenter()).and(hc.cross   ()).onTrue(goToPositionCommand[ 2]);// Low Foward Coral: 2
      noTriggersOrBumpers.and(hc.povRight ()).and(hc.cross   ()).onTrue(goToPositionCommand[ 3]);// Low Back Coral: 3
      noTriggersOrBumpers.and(hc.povDown  ()).and(hc.circle  ()).onTrue(goToPositionCommand[ 3]);// Low Reef Ball: 4
      noTriggersOrBumpers.and(hc.povCenter()).and(hc.square  ()).onTrue(goToPositionCommand[ 5]);// Mid Foward Coral: 5
      noTriggersOrBumpers.and(hc.povRight ()).and(hc.square  ()).onTrue(goToPositionCommand[ 6]);// Mid Back Coral: 6
      noTriggersOrBumpers.and(hc.povUp    ()).and(hc.circle  ()).onTrue(goToPositionCommand[ 7]);// High Reef Ball: 7
      noTriggersOrBumpers.and(hc.povCenter()).and(hc.triangle()).onTrue(goToPositionCommand[ 8]);// High Foward Coral: 8
      noTriggersOrBumpers.and(hc.povRight ()).and(hc.triangle()).onTrue(goToPositionCommand[ 9]);// High Back Coral: 9

      noTriggersOrBumpers.and(hc.povUp    ()).and(hc.cross   ()).onTrue(goToPositionCommand[16]);// Coral from Human
      
      hc.L1().and(hc.R1()).and(hc.L2().and(hc.R2()).negate())   .onTrue (goToPositionCommand[10]);// climb ready
      hc.L1().and(hc.R1()).and(hc.L2().and(hc.R2()).negate())   .onFalse(goToPositionCommand[11]);// climbing
      noTriggersOrBumpers.and(hc.button(12))             .onTrue (goToPositionCommand[19]);// Intermediate Position: 19

      hc.L2().and(hc.triangle()).onTrue(grabCommand);
      hc.L2().and(hc.cross()).onTrue(releaseCommand);
      hc.L1().and(hc.L2()).onTrue(new InstantCommand(()->climb.set(1)));
      hc.L1().and(hc.L2()).onFalse(new InstantCommand(()->climb.set(0)));

    }
    
    /**
     * sets default commands for systems
     * configures all position commands
     * configures bounds
     * applies encoder offsets
     * register arm commands to pathplanner
     */
    private void configureArmSystems(){

      shoulder1.setDefaultCommand(new JointToPosition(shoulder1));
      // shoulder2.setDefaultCommand(new JointToPosition(shoulder2));
      wrist    .setDefaultCommand(new JointToPosition(wrist    ));
      telescope.setDefaultCommand(new JointToPosition(telescope));

      if(ArmConstants.useBounds){
        shoulder1.applyBounds(ArmConstants.shoulderMin , ArmConstants.shoulderMax );
        // shoulder2.applyBounds(ArmConstants.shoulderMin , ArmConstants.shoulderMax );
        telescope.applyBounds(ArmConstants.telescopeMin, ArmConstants.telescopeMax);
        wrist    .applyBounds(ArmConstants.wristMax    , ArmConstants.wristMax    );
      }

      shoulder1.getEncoder().setOffset(ArmConstants.shoulderOffset /360);
      telescope.getEncoder().setOffset(ArmConstants.telescopeOffset/360);
      wrist    .getEncoder().setOffset(ArmConstants.wristOffset    /360);

      shoulder1.setSetpoint(shoulder1.getAngleDegrees());
      // shoulder2.setSetpoint(shoulder2.getAngleDegrees());
      telescope.setSetpoint(telescope.getAngleDegrees());
      wrist    .setSetpoint(wrist    .getAngleDegrees());
      
      grabber.getCoralMotor().setNeutralMode(Motor.idleMode.Brake);
      wrist.getMotor().setNeutralMode(idleMode.Brake);
      // telescope.calibrateWithColors();

      // defining all the setpoint commands
      for(int i = 0; i<ArmConstants.positions.length; i++){
        ArmPosition thisArmPosition = ArmConstants.positions[i];
        goToPositionCommand[i] = new InstantCommand(()->{
          wrist    .setSetpoint(thisArmPosition.getWristPos    ());
          shoulder1.setSetpoint(thisArmPosition.getShoulderPos ());
          // shoulder2.setSetpoint(thisArmPosition.getShoulderPos ());
          telescope.setSetpoint(thisArmPosition.getTelescopePos());
          SmartDashboard.putString("Arm Setpoint", thisArmPosition.getName());
        });
        NamedCommands.registerCommand(thisArmPosition.getName(), goToPositionCommand[i]);
      }
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
      // An example command will be run in autonomous
      return swerve.getResetGyro().andThen(swerve.getAutonomousCommand(OperatorConstants.AutonomousCommandName));
    }
    
    public void periodic(){
      Rotation3d rot = swerve.getGyro().getRawRotation3d();
      SmartDashboard.putNumberArray("Heading via RobotContainer", new double[] {rot.getX(),rot.getY(),rot.getZ()});
      // swerve.getHeading();
      SmartDashboard.putNumber("Match Time Remaining", Timer.getMatchTime());
    }

    public SwerveSubsystem getSwerve() {
      return swerve;
    }
    public static CommandGenericHID getController() {
      return m_driverController;
  }
}
