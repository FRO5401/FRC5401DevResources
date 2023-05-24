package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import frc.robot.Utilities.SparkMAXMotorGroup;
import frc.robot.Utilities.SparkMaxPIDConfig;

public class Hardware {

    public CANSparkMax leftDrive1;
	public CANSparkMax leftDrive2;
	public CANSparkMax leftDrive3;
	public CANSparkMax rightDrive1;
	public CANSparkMax rightDrive2;
	public CANSparkMax rightDrive3;

    public RelativeEncoder leftDrive1Enc;
    public RelativeEncoder leftDrive2Enc;
    public RelativeEncoder rightDrive1Enc;
    public RelativeEncoder rightDrive2Enc;

    public SparkMAXMotorGroup leftDriveMotors;
    public SparkMAXMotorGroup rightDriveMotors;

    public Solenoid firstClawStage; 
    public Solenoid secondClawStage; 

    public CANSparkMax leftArm;
    public CANSparkMax rightArm;
    public CANSparkMax transArm;

    public SparkMaxPIDConfig leftArmPIDConfig;
    public SparkMaxPIDConfig rightArmPIDConfig;
    public SparkMaxPIDConfig transArmPIDConfig;

    public SparkMaxPIDController leftArmPID;
    public SparkMaxPIDController rightArmPID;
    public SparkMaxPIDController transArmPID;

    public RelativeEncoder leftArmEnc;
    public RelativeEncoder rightArmEnc;
    public RelativeEncoder transArmEnc;

    public CANSparkMax leftShooterMotor;
    public CANSparkMax rightShooterMotor;

    public SparkMaxPIDController leftShooterPID;
    public SparkMaxPIDController rightShooterPID;

    public RelativeEncoder leftShooterEnc;
    public RelativeEncoder rightShooterEnc;
  

   


    public Hardware() {
        //Drivebase hardware
        leftDrive1 = new CANSparkMax(0, MotorType.kBrushless);
        leftDrive2 = new CANSparkMax(1, MotorType.kBrushless);
        rightDrive1 = new CANSparkMax(2, MotorType.kBrushless);
        rightDrive2 = new CANSparkMax(3, MotorType.kBrushless);

        leftDrive1Enc = leftDrive1.getEncoder();
        leftDrive2Enc = leftDrive1.getEncoder();
        rightDrive1Enc = leftDrive1.getEncoder();
        rightDrive2Enc = leftDrive1.getEncoder();

        leftDriveMotors = new SparkMAXMotorGroup("Left Drive Motor Group", leftDrive1, leftDrive2);
        rightDriveMotors = new SparkMAXMotorGroup("Right Drive Motor Group", rightDrive1, rightDrive2);

        leftDriveMotors.setIdleMode(IdleMode.kBrake);
        rightDriveMotors.setIdleMode(IdleMode.kBrake);

        //Claw hardware
        firstClawStage = new Solenoid(PneumaticsModuleType.CTREPCM, 1);
        secondClawStage = new Solenoid(PneumaticsModuleType.CTREPCM, 2);

        //Simple arm hardware
        leftArm = new CANSparkMax(4, MotorType.kBrushless);
        rightArm = new CANSparkMax(5, MotorType.kBrushless);
        transArm = new CANSparkMax(6, MotorType.kBrushless);

        leftArmPIDConfig = new SparkMaxPIDConfig("Left Arm PID Controler", leftArm.getPIDController(), 0, 0, 0, 0);
        rightArmPIDConfig = new SparkMaxPIDConfig("Left Arm PID Controler", rightArm.getPIDController(), 0, 0, 0, 0);
        transArmPIDConfig = new SparkMaxPIDConfig("Left Arm PID Controler", transArm.getPIDController(), 0, 0, 0, 0);

        leftArmPID = leftArmPIDConfig.getConfPIDController();
        rightArmPID = rightArmPIDConfig.getConfPIDController();
        transArmPID = transArmPIDConfig.getConfPIDController();
        
        leftArmEnc = leftArm.getEncoder();
        rightArmEnc = rightArm.getEncoder();
      


    }


    



}
