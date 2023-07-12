package frc.robot.Subsystems;

import frc.robot.Robot;
import frc.robot.Utilities.SparkMAXMotorGroup;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
//import frc.robot.Constants;

/**
 * Add your docs here.
 */
public class ExampleSimpleElevator extends SubsystemBase {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  private CANSparkMax leftElevatorMotor = Robot.hardware.leftElevatorMotor; //Initiating the elevator motors
  private CANSparkMax rightElevatorMotor = Robot.hardware.rightElevatorMotor;

  private SparkMaxPIDController leftElevatorPID = Robot.hardware.leftElevatorPID; //Initiating the PID motors
  private SparkMaxPIDController rightElevatorPID = Robot.hardware.rightElevatorPID;

  private RelativeEncoder leftElevatorEnc = Robot.hardware.leftElevatorEnc; //Initiating the relative encoders
  private RelativeEncoder rightElevatorEnc = Robot.hardware.rightElevatorEnc;


  public class ElevatorState { //Methods for the 2 positions
    private double leftPos;
    private double rightPos;

    public ElevatorState(double leftPos, double rightPos){ 
      this.leftPos = leftPos; //Sets the constructors
      this.rightPos = rightPos;
    };
    
  }

  public ElevatorState currentElevatorState; 


<<<<<<< Updated upstream
  Map<String, ElevatorState> elevatorStates = Map.ofEntries(
                    Map.entry("DEFAULT", new ElevatorState(0, 0)),
                    Map.entry("LOW_HEIGHT", new ElevatorState(5, 5)), 
                    Map.entry("HIGH_HEIGHT", new ElevatorState(10, 10))
=======
  Map<String, ElevatorState> elevatorStates = Map.ofEntries( 
                    Map.entry("DEFAULT", new ElevatorState(0, 0)), //Setting default to 0, 0
                    Map.entry("LOW_HEIGHT", new ElevatorState(5, 5)), //Setting low_height to 5, 5
                    Map.entry("HIGH_HEIGHT", new ElevatorState(10, 10)) //Setting high_height to 10, 10
>>>>>>> Stashed changes

                );
        
  public ExampleSimpleElevator() {
    
    leftElevatorMotor.setInverted(true); //Inverting the left motor
    leftElevatorMotor.follow(rightElevatorMotor); //Sets the left motor to follow the right
  }

  public void setPivotSpeed(double power) { //Sets the power of pivot
    rightElevatorMotor.set(power);
    SmartDashboard.putNumber("Left Arm Output ", power);
    SmartDashboard.putNumber( "Right Arm Output ", power);    
}

  public void setPositions(ElevatorState elevatorState){ //Sets the position of the elevator
    currentElevatorState = elevatorState;
    double leftSetpoint = elevatorState.leftPos;
    double rightSetpoint = elevatorState.rightPos;

    leftElevatorPID.setReference(leftSetpoint, ControlType.kSmartMotion); //Sets a reference point for PID
    rightElevatorPID.setReference(rightSetpoint, ControlType.kSmartMotion);

    SmartDashboard.putNumber("Right Setpoint", rightSetpoint);
    SmartDashboard.putNumber("Left Setpoint", leftSetpoint);
  }


  public ElevatorState getElevatorState(){ //Returns elevator state
    return currentElevatorState;
  }
  public List<Double> getPositions(){ //Uses an array to get positions of the elevator encoders
    List<Double> positions = new ArrayList<Double>(3);
    positions.add(leftElevatorEnc.getPosition());
    positions.add(rightElevatorEnc.getPosition());

    //Average velocity of the two shooter motors for indication of shared axle problems
    positions.add((rightElevatorEnc.getPosition() + leftElevatorEnc.getPosition()) / 2); 
     return positions;
  }


}