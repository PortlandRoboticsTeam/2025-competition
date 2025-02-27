package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Encoder;
import frc.robot.subsystems.Joint;

public class JointToPosition extends Command{
    Joint m_joint;
    Encoder encoder;

    public JointToPosition(Joint joint) {
    m_joint = joint;
    encoder = joint.getEncoder();

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_joint);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(m_joint.isPIDEnabled())
      if (encoder.isConnected()) {
        double output = m_joint.getController().calculate(m_joint.getAngleDegrees(), m_joint.getSetpoint());
        m_joint.setSpeed(output);
      }else{
        m_joint.stop();
      }
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_joint.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}