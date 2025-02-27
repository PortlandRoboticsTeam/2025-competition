package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.ArmPosition;
import frc.robot.subsystems.Joint;
import frc.robot.subsystems.Telescope;

public class ArmPositionCommandGenerator {
  private Joint shoulder1;
  private Joint shoulder2;
  private Telescope telescope;
  private Joint wrist;

  private final ArmPosition tol = ArmConstants.positionCommandCompletionTolerance;

  public ArmPositionCommandGenerator(Joint s1, Joint s2, Telescope t, Joint w){
    shoulder1=s1; shoulder2=s2; telescope=t; wrist=w;
  }

  public Command compileCommand(ArmPosition pos){
    Command shoulderCommand =  shoulder1.getGoToCommand(pos.getShoulderPos (), tol.getShoulderPos ())
                    .alongWith(shoulder2.getGoToCommand(pos.getShoulderPos (), tol.getShoulderPos ()));
    Command wristCommand =     wrist    .getGoToCommand(pos.getWristPos    (), tol.getWristPos    ());
    Command telescopeCommand = telescope.getGoToCommand(pos.getTelescopePos(), tol.getTelescopePos());

    return wristCommand.andThen(
      pos.getShoulderPos() > shoulder1.getAngleDegrees() ?
        shoulderCommand.andThen(telescopeCommand) :
        telescopeCommand.andThen(shoulderCommand)
    );
  }
}
