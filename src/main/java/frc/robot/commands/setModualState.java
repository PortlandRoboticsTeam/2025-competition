package frc.robot.commands;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.PosidenModual;

public class setModualState extends Command{
    PosidenModual modual;
    DoubleSupplier speed;
    DoubleSupplier angle;
    public setModualState(PosidenModual modual,DoubleSupplier speed,DoubleSupplier angle){
        this.modual = modual;
        this.speed = speed;
        this.angle = angle;
        addRequirements(modual);
    }
    @Override
    public void execute(){
        modual.setDesiredState(new SwerveModuleState(speed.getAsDouble(), new Rotation2d(angle.getAsDouble()*Math.PI/2)));

    }
    
    // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted)
  {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished()
  {
    return true;
  }
}
