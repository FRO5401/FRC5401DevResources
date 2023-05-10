package frc.robot.Utilities;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;


public class SparkMaxPIDConfig {
	private SparkMaxPIDController motorPID;
	private String name;
	private double kP;
	private double kI;
	private double kD;

	public SparkMaxPIDConfig(String name, SparkMaxPIDController motorPID, double kP, double kI, double kD, double kFF) {
		
		this.name = name;
		this.motorPID = motorPID;
		motorPID.setP(kP);
		motorPID.setI(kI);
		motorPID.setD(kD);	
		motorPID.setFF(kFF);

	}

	public SparkMaxPIDController getConfPIDController(){
		return motorPID;
	}
	
}