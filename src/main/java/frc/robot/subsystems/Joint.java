package frc.robot.subsystems;

// Import necessary libraries for hardware control and PID management
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


// Joint class extends SubsystemBase, allowing integration into the FRC command-based framework
public class Joint extends SubsystemBase {
    private PIDController pid; // PID controller for managing motor position
    private CANcoder encoder; // Encoder to read joint position
    private TalonFX motor; // Motor controlling the joint movement
    private int[] setpoints; // Array of predefined setpoints for the joint
    private int position; // Current position index in the setpoints array
    private int jointNum; // ID of the joint
    private boolean inverted; // Weather or not the encoder is inverted

    // Constructor to initialize the Joint subsystem
    public Joint(int jointNum, int motorID, int encoderID, boolean inverted, int defaultSetpointIndex, int[] setpoints, double kP, double kI, double kD) {
        // Initialize PID controller with specified gains
        pid = new PIDController(kP, kI, kD);
        // Enable continuous input for angles, wrapping around at -180 to 180 degrees
        pid.enableContinuousInput(-180, 180);
        // Initialize encoder with the given encoder ID
        encoder = new CANcoder(encoderID);
        // Initialize motor with the given motor ID and specify it as brushless
        motor = new TalonFX(motorID);
        // Store the setpoints and initialize the current position
        this.setpoints = setpoints;
        position = defaultSetpointIndex;
        // Initalize the jointNum
        this.jointNum = jointNum;
        // Initalized inverted
        this.inverted = inverted;
    }

    // Periodic method called every scheduler loop; can be used for periodic tasks
    @Override
    public void periodic() {
        // Currently empty; can be used to update state or perform checks
        SmartDashboard.putNumber("current angel for joint number "+jointNum, getAngleDegrees());
        SmartDashboard.putNumber("current Position Number for joint number "+jointNum, position);
        SmartDashboard.putNumber("current setPoint Degrees for joint number "+jointNum, setpoints[position]);
        SmartDashboard.putString("joint "+jointNum+" info", toString());
        
    }

    // Method to set the current position index
    public void setPosition(int position) {
        this.position = position;
    }

    // Method to get the current position index
    public int getPosition() {
        return position;
    }

    // Method to set the motor speed as a percentage (from -1.0 to 1.0)
    public void setSpeed(double speedPercentage) {
        double output;
        if(inverted){
            output = Math.max(-1.0, Math.min(1.0, -speedPercentage));
            SmartDashboard.putNumber("Joint "+getID()+" output", output);
        }else{
            output = Math.max(-1.0, Math.min(1.0, speedPercentage));
            SmartDashboard.putNumber("Joint "+getID()+" output", output);
        }
        motor.set(output);
    }

    // Method to get the encoder's position in terms of rotations
    public double getRotations() {
        return encoder.getPosition().getValueAsDouble();
    }

    // Method to get the angle in degrees based on encoder position
    public double getAngleDegrees() {
        return encoder.getPosition().getValueAsDouble() * 360 % 360;
    }

    // Method to get the angle in radians based on encoder position
    public double getAngleRadians() {
        return encoder.getPosition().getValueAsDouble() * 2 * Math.PI;
    }

    // Method to get the PID controller
    public PIDController getController() {
        return pid;
    }

    // Method to get the PID controller
    public int getID() {
        return jointNum;
    }

    // Method to get the array of setpoints
    public int[] getSetpoints() {
        return setpoints;
    }

    // Method to stop motor
    public void stop() {
        setSpeed(0); // Method to stop the motor
    }

    @Override
    public String toString(){
        String start = "Joint number:"+jointNum+"\n position number: "+position+"\n positions: ";
        String posString = "{";
        for (int i : setpoints) {
            posString = posString+i+", ";              
        }
        posString = posString.substring(0,posString.length()-2)+"}\n";
        String pidString = "P: "+pid.getP()+"\nI: "+pid.getI()+"\nD: "+pid.getD()+"\n";
        String currentPos = "current angle:"+encoder.getPosition()+"\n";
        return start+posString+pidString+currentPos;
    }
    public void incrementPosition(){
        if(position<setpoints.length-1){
            position++;
        }else{
            position = 0;
        }
    }
}

