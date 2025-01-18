package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Joint;

public class JointToPosition extends Command{
    Joint m_joint;
    int[] setpoints;

    public JointToPosition(Joint joint) {
    m_joint = joint;
    setpoints = m_joint.getSetpoints();

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_joint);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double output = m_joint.getController().calculate(m_joint.getAngleDegrees(), setpoints[m_joint.getPosition()]);
    
    m_joint.setSpeed(output);
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
