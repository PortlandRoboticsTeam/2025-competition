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
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.Telescope;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import com.pathplanner.lib.auto.NamedCommands;
// import com.studica.frc.AHRS;
// import com.studica.frc.AHRS.NavXComType;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  
  
  // hardware objects vv
  private final SwerveSubsystem swerve = new SwerveSubsystem();
  Joint shoulder1     = new Joint    (0, ArmConstants.shoulder1ID, ArmConstants.shoulderEncoderID , false, 0, 0.13, 0, 0.001, ArmConstants.shoulder1Type, ArmConstants.shoulderEncoderType);
  Joint shoulder2     = new Joint    (0, ArmConstants.shoulder2ID, shoulder1.getEncoder()         , false, 0, 0.13, 0, 0.001, ArmConstants.shoulder2Type);
  Joint wrist         = new Joint    (1, ArmConstants.wristID    , ArmConstants.wristEncoderID    , false, 0, 0.005, 0, 0, ArmConstants.wristType    , ArmConstants.wristEncoderType    );
  Telescope telescope = new Telescope(2, ArmConstants.telescopeID, ArmConstants.telescopeEncoderID, true , 0, 0.65, 0, 0, ArmConstants.telescopeType, ArmConstants.telescopeEncoderType);
  // ArmPositionCommandGenerator armCommandGenerator = new ArmPositionCommandGenerator(shoulder1, shoulder2, telescope, wrist);
  Grabber grabber = new Grabber();
  PIDController steeringController = new PIDController(.03, 0, 0);
  // AHRS navx = new AHRS(NavXComType.kMXP_SPI);


  // controllers vv
  private final static CommandPS4Controller m_driverController =
        new CommandPS4Controller(OperatorConstants.driverControllerPort);
  private final static CommandPS4Controller m_helperController =
        new CommandPS4Controller(OperatorConstants.helperControllerPort);
  
  
  // variables vv
  private Command driveCommand; 
  private boolean isFieldOriented = true;
  private double steeringTargetAngle = 0;
  private boolean doMaintainAngle = true;
  private Command visdrive = swerve.vishionDrive();


  // commands vv
  Command resetGyro = swerve.getResetGyro().andThen(()->{steeringTargetAngle=0;});
  InstantCommand robotOrientCommand = new InstantCommand(()->{ isFieldOriented = false; });
  InstantCommand fieldOrientCommand = new InstantCommand(()->{ isFieldOriented = true ; });
  InstantCommand[] goToPositionCommand = new InstantCommand[ArmConstants.positions.length];
  InstantCommand grabCommand = new InstantCommand(()->grabber.grab());
  InstantCommand releaseCommand = new InstantCommand(()->grabber.release());
  InstantCommand stopGrabbingCommand = new InstantCommand(()->grabber.stop());
  GenericCommand runArmManualCommand = new GenericCommand(
    ()->{
      shoulder1.disablePID();
      shoulder2.disablePID();
      // wrist    .disablePID();
      telescope.disablePID();
    },
    ()->{
      shoulder1.setSpeed(MathUtil.applyDeadband(-m_helperController.getLeftY (), ArmConstants.manualControlJoystickDeaband));
      shoulder2.setSpeed(MathUtil.applyDeadband(-m_helperController.getLeftY (), ArmConstants.manualControlJoystickDeaband));
      // wrist    .setSpeed(MathUtil.applyDeadband(-m_helperController.getLeftX (), ArmConstants.manualControlJoystickDeaband)/5);
      telescope.setSpeed(MathUtil.applyDeadband(-m_helperController.getRightY(), ArmConstants.manualControlJoystickDeaband));
      // shoulder1.setSetpoint(shoulder1.getSetpoint() + MathUtil.applyDeadband(-m_helperController.getLeftY (), ArmConstants.manualControlJoystickDeaband));
      // shoulder2.setSetpoint(shoulder1.getSetpoint());
      wrist    .setSetpoint(wrist    .getSetpoint() + MathUtil.applyDeadband( m_helperController.getLeftX (), ArmConstants.manualControlJoystickDeaband));
      // telescope.setSetpoint(telescope.getSetpoint() - 0.1*MathUtil.applyDeadband( m_helperController.getRightY(), ArmConstants.manualControlJoystickDeaband));
    },
    ()->{
      shoulder1.setSetpoint(shoulder1.getAngleDegrees());
      shoulder2.setSetpoint(shoulder1.getAngleDegrees());
      wrist    .setSetpoint(wrist    .getAngleDegrees());
      telescope.setSetpoint(telescope.getAngleDegrees());
      shoulder1.enablePID();
      shoulder1.enablePID();
      wrist    .enablePID();
      if(!telescope.isCalibrating()) telescope.enablePID();
    }
  );
  GenericCommand steeringCommand = new GenericCommand(
    ()->doMaintainAngle=false, 
    ()->{}, 
    ()->{ steeringTargetAngle = swerve.getHeading().getDegrees(); doMaintainAngle=true; }
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
        () ->  doMaintainAngle ? 
                  steeringController.calculate(steeringTargetAngle, swerve.getHeading().getDegrees()) : 
                  MathUtil.applyDeadband(m_helperController.getL2Axis() > 0.5 ? 
                  m_helperController.getRightX() : 
                  m_driverController.getRawAxis(2), Constants.DEADBAND),
        () ->  isFieldOriented
      );
      
      swerve.setDefaultCommand(driveCommand);

      steeringController.enableContinuousInput(0, 360);

      configureArmSystems();
      registerGrabberCommands();
      configureBindings();
      
    }
    private void registerGrabberCommands(){
      NamedCommands.registerCommand("grab", grabCommand);
      NamedCommands.registerCommand("release", releaseCommand);
      NamedCommands.registerCommand("stopGrabbing", stopGrabbingCommand);
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
      m_driverController.square().onTrue(resetGyro);
      m_driverController.L1().onFalse(fieldOrientCommand);
      m_driverController.L1().onTrue(robotOrientCommand);

      m_driverController.axisMagnitudeGreaterThan(2, Constants.DEADBAND).whileTrue(steeringCommand);
      m_driverController.R2().onTrue(grabCommand);
      m_driverController.L2().onTrue(releaseCommand);
      m_driverController.button(12).onTrue(visdrive); // rstick button
      m_driverController.button(12).onFalse(driveCommand);
      m_driverController.R1().whileTrue(new InstantCommand(()->{doMaintainAngle=false;}));
      m_driverController.R1().whileFalse(new InstantCommand(()->{doMaintainAngle=true;}));

      CommandPS4Controller hc = m_helperController;

      // see the control guide:
      // https://docs.google.com/presentation/d/1Df-ZnbTBpNsHdl3szxJ4QNOpd_V1Ua5YG6Antzs98B0/edit#slide=id.g2af72db80bc_0_5

      // rest and intermediate
      hc.R1().and(hc.cross()).onTrue(goToPositionCommand[ 0]);
      hc.button(12).onTrue(goToPositionCommand[19]);

      // reef scoring:   (1-9)
      hc.cross   ().and(hc.povDown ()).onTrue(goToPositionCommand[ 1]);
      hc.cross   ().and(hc.povRight()).onTrue(goToPositionCommand[ 2]);
      hc.cross   ().and(hc.povLeft ()).onTrue(goToPositionCommand[ 3]);
      hc.circle  ().and(hc.povDown ()).onTrue(goToPositionCommand[ 4]);
      hc.square  ().and(hc.povRight()).onTrue(goToPositionCommand[ 5]);
      hc.square  ().and(hc.povLeft ()).onTrue(goToPositionCommand[ 6]);
      hc.circle  ().and(hc.povUp   ()).onTrue(goToPositionCommand[ 7]);
      hc.triangle().and(hc.povRight()).onTrue(goToPositionCommand[ 8]);
      hc.triangle().and(hc.povLeft ()).onTrue(goToPositionCommand[ 9]);

      // floor pickup:  (12-15)
      // hc.povDown().and(hc.L2()).and(hc.circle()).onTrue(goToPositionCommand[12]);
      // hc.povDown().and(hc.L2()).and(hc.cross ()).onTrue(goToPositionCommand[13]);
      // hc.povUp  ().and(hc.L2()).and(hc.circle()).onTrue(goToPositionCommand[14]);
      // hc.povUp  ().and(hc.L2()).and(hc.cross ()).onTrue(goToPositionCommand[15]);

      // other positions:  (16-18)
      // hc.L1().and(
        hc.povUp  ().and(hc.cross ()).onTrue(goToPositionCommand[16]);
      // hc.L1().and(hc.povDown()).and(hc.circle()).onTrue(goToPositionCommand[17]);
      // hc.L1().and(hc.povUp  ()).and(hc.circle()).onTrue(goToPositionCommand[18]);
      
      // rest position (0), zero gyro, and manual arm
      hc.cross().and(hc.R2()).onTrue(goToPositionCommand[ 0]);
      hc.square().and(hc.R2()).onTrue(resetGyro);
      hc.R2().whileTrue(runArmManualCommand);

      // climb controls (10 & 11)
      hc.L1().and(hc.R1()).onTrue (goToPositionCommand[10]);
      hc.L1().and(hc.R1()).onFalse(goToPositionCommand[11].andThen(()->{doMaintainAngle=false;}));
      hc.R2().and(hc.triangle()).onTrue(calibrateTelescope);
    
    }
    
    private void configureOldBindings() {
      m_driverController.button(11).onTrue(resetGyro);
      m_driverController.button(3).whileTrue(swerve.vishionDrive());
      m_driverController.button(5).onFalse(fieldOrientCommand);
      m_driverController.button(5).onTrue(robotOrientCommand);

      // m_driverController.button(1).onTrue (goToPositionCommand[11]);
      // m_driverController.button(1).onFalse(goToPositionCommand[10]);

      m_driverController.axisMagnitudeGreaterThan(2, Constants.DEADBAND).whileTrue(steeringCommand);
      m_driverController.button(1).onTrue(grabCommand);
      m_driverController.button(2).onTrue(releaseCommand);
      m_driverController.button(8).onTrue(visdrive);
      m_driverController.button(8).onFalse(driveCommand);
      m_driverController.button(10).whileTrue(new InstantCommand(()->{doMaintainAngle=false;}));
      m_driverController.button(10).whileFalse(new InstantCommand(()->{doMaintainAngle=true;}));

      //m_driverController.button(1).or(m_driverController.button(2)).onFalse(stopGrabbingCommand);

      CommandPS4Controller hc = m_helperController;

      // see the control guide:
      // https://docs.google.com/presentation/d/1Df-ZnbTBpNsHdl3szxJ4QNOpd_V1Ua5YG6Antzs98B0/edit#slide=id.g2af72db80bc_0_5

      // rest and intermediate
      hc.R1().and(hc.cross()).onTrue(goToPositionCommand[ 0]);
      hc.button(12).onTrue(goToPositionCommand[19]);

      // reef scoring:   (1-9)
      hc.cross   ().and(hc.povDown ()).onTrue(goToPositionCommand[ 1]);
      hc.cross   ().and(hc.povRight()).onTrue(goToPositionCommand[ 2]);
      hc.cross   ().and(hc.povLeft ()).onTrue(goToPositionCommand[ 3]);
      hc.circle  ().and(hc.povDown ()).onTrue(goToPositionCommand[ 4]);
      hc.square  ().and(hc.povRight()).onTrue(goToPositionCommand[ 5]);
      hc.square  ().and(hc.povLeft ()).onTrue(goToPositionCommand[ 6]);
      hc.circle  ().and(hc.povUp   ()).onTrue(goToPositionCommand[ 7]);
      hc.triangle().and(hc.povRight()).onTrue(goToPositionCommand[ 8]);
      hc.triangle().and(hc.povLeft ()).onTrue(goToPositionCommand[ 9]);

      // floor pickup:  (12-15)
      // hc.povDown().and(hc.L2()).and(hc.circle()).onTrue(goToPositionCommand[12]);
      // hc.povDown().and(hc.L2()).and(hc.cross ()).onTrue(goToPositionCommand[13]);
      // hc.povUp  ().and(hc.L2()).and(hc.circle()).onTrue(goToPositionCommand[14]);
      // hc.povUp  ().and(hc.L2()).and(hc.cross ()).onTrue(goToPositionCommand[15]);

      // other positions:  (16-18)
      hc.L1().and(hc.povUp  ()).and(hc.cross ()).onTrue(goToPositionCommand[16]);
      // hc.L1().and(hc.povDown()).and(hc.circle()).onTrue(goToPositionCommand[17]);
      // hc.L1().and(hc.povUp  ()).and(hc.circle()).onTrue(goToPositionCommand[18]);
      
      // rest position (0), zero gyro, and manual arm
      hc.cross().and(hc.R2()).onTrue(goToPositionCommand[ 0]);
      hc.square().and(hc.R2()).onTrue(resetGyro);
      hc.R2().whileTrue(runArmManualCommand);

      // climb controls (10 & 11)
      hc.L1().and(hc.R1()).onTrue (goToPositionCommand[10]);
      hc.L1().and(hc.R1()).onFalse(goToPositionCommand[11].andThen(()->{doMaintainAngle=false;}));
      hc.R2().and(hc.triangle()).onTrue(calibrateTelescope);
    
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
      shoulder2.setDefaultCommand(new JointToPosition(shoulder2));
      wrist    .setDefaultCommand(new JointToPosition(wrist    ));
      telescope.setDefaultCommand(new JointToPosition(telescope));
      wrist.getPID().disableContinuousInput();

      if(ArmConstants.useBounds){
        shoulder1.applyBounds(ArmConstants.shoulderMin , ArmConstants.shoulderMax );
        shoulder2.applyBounds(ArmConstants.shoulderMin , ArmConstants.shoulderMax );
        telescope.applyBounds(ArmConstants.telescopeMin, ArmConstants.telescopeMax);
        wrist    .applyBounds(ArmConstants.wristMax    , ArmConstants.wristMax    );
      }

      shoulder1.getEncoder().setOffset(ArmConstants.shoulderOffset /360);
      telescope.getEncoder().setOffset(ArmConstants.telescopeOffset/360);
      wrist    .getEncoder().setOffset(ArmConstants.wristOffset    /360);

      shoulder1.setSetpoint(shoulder1.getAngleDegrees());
      shoulder2.setSetpoint(shoulder2.getAngleDegrees());
      telescope.setSetpoint(telescope.getAngleDegrees());
      wrist    .setSetpoint(wrist    .getAngleDegrees());
      

      // telescope.calibrateWithColors();

      // defining all the setpoint commands
      for(int i = 0; i<ArmConstants.positions.length; i++){
        ArmPosition thisArmPosition = ArmConstants.positions[i];
        goToPositionCommand[i] = new InstantCommand(()->{
          wrist    .setSetpoint(thisArmPosition.getWristPos    ());
          shoulder1.setSetpoint(thisArmPosition.getShoulderPos ());
          shoulder2.setSetpoint(thisArmPosition.getShoulderPos ());
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
      return swerve.getAutonomousCommand(OperatorConstants.AutonomousCommandName);
    }
    
    public void periodic(){ }

    public SwerveSubsystem getSwerve() {
      return swerve;
    }
    public static CommandGenericHID getController() {
      return m_driverController;
  }
}
