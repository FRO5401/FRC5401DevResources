package frc.robot.Utilities;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;


public class SparkMaxPIDConfig {
	private SparkMaxPIDController motorPID;
	private String name;
	private double kP; //Proportional
	private double kI; //Integral
	private double kD; //Derivative

	//Creates the constructors
	public SparkMaxPIDConfig(String name, SparkMaxPIDController motorPID, double kP, double kI, double kD, double kFF) {
		
		this.name = name;
		this.motorPID = motorPID;
		motorPID.setP(kP); //Sets the proportional constructor
		motorPID.setI(kI); //Sets the integral constructor
		motorPID.setD(kD); //Sets the derivative constructor
		motorPID.setFF(kFF);

	}

	public SparkMaxPIDController getConfPIDController(){
		return motorPID;
	}
	
}