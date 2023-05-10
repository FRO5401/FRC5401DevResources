package frc.robot.Utilities;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;


public class SparkMAXMotorGroup {
	private CANSparkMax masterMotor;
	private CANSparkMax[] motors;
	private String name;

	public SparkMAXMotorGroup(String name, CANSparkMax masterMotor, CANSparkMax... motors) {
		
		this.name = name;
		this.masterMotor = masterMotor;
		this.motors = motors;
		setup(masterMotor);
		for (CANSparkMax element : motors) {
			setup(element);
		}
	}

	
	public void setup(CANSparkMax m_motor) {

		m_motor.restoreFactoryDefaults();

    }

	public void set(double power) {
		masterMotor.set(power);
		for (CANSparkMax motor : this.motors) {
			motor.set(power);
		}
	}

	public void stop() {
		set(0);
	}

	public void setIdleMode(IdleMode idleMode) {
		for (CANSparkMax motor : this.motors) {
			motor.setIdleMode(idleMode);
		}
	}

	public void setInverted(boolean isInverted) {
		masterMotor.setInverted(isInverted);
		for (CANSparkMax motor : this.motors) {
			motor.setInverted(isInverted);
		}
	}
	public CANSparkMax getMasterMotor() {
		return masterMotor;
	}

	public double getEncoderPosition() {
		return masterMotor.getEncoder().getPosition();
	}
}