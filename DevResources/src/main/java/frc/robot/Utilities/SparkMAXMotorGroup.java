package frc.robot.Utilities;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;


public class SparkMAXMotorGroup {
	private CANSparkMax masterMotor; //Methods for the motors
	private CANSparkMax[] motors;
	private String name; //Creates a String

	public SparkMAXMotorGroup(String name, CANSparkMax masterMotor, CANSparkMax... motors) {
		
		this.name = name; //Sets the constructors
		this.masterMotor = masterMotor;
		this.motors = motors;
		setup(masterMotor);
		for (CANSparkMax element : motors) {
			setup(element);
		}
	}

	
	public void setup(CANSparkMax m_motor) { //Sets the motors to default before starting power

		m_motor.restoreFactoryDefaults();

    }

	public void set(double power) { //Sets power to the motors
		masterMotor.set(power);
		for (CANSparkMax motor : this.motors) {
			motor.set(power);
		}
	}

	public void setVoltages(double power) { //Sets voltage to power
		masterMotor.setVoltage(power);
		for (CANSparkMax motor : this.motors) {
			motor.setVoltage(power);
		}
	}


	public void stop() { //Sets it so there is no power
		set(0);
	}

	public void setIdleMode(IdleMode idleMode) { //Sets the masterMotor idle
		masterMotor.setIdleMode(idleMode);
		for (CANSparkMax motor : this.motors) {
			motor.setIdleMode(idleMode);
		}
	}

	public void setInverted(boolean isInverted) { //Sets the masterMotor to be inverted or not
		masterMotor.setInverted(isInverted);
		for (CANSparkMax motor : this.motors) {
			motor.setInverted(isInverted);
		}
	}
	public CANSparkMax getMasterMotor() { //Returns the masterMotor
		return masterMotor;
	}

	public double getEncoderPosition() { //Returns the masterMotor encoder position
		return masterMotor.getEncoder().getPosition();
	}
}