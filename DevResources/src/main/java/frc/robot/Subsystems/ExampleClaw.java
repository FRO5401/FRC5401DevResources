package frc.robot.Subsystems;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

public class ExampleClaw extends SubsystemBase {
    private Solenoid firstStage;
    private Solenoid secondStage;
    private boolean firstStateStageStatus;
    private boolean secondStateStageStatus;


    public ExampleClaw(){
        firstStage = Robot.hardware.firstClawStage;
        secondStage = Robot.hardware.secondClawStage;
    }

    public void setSolenoidStatuses(String mode){
        switch(mode){
            case "HALF_COMPRESSION":
                firstStateStageStatus = true;
                secondStateStageStatus = false;

            break;
            case "FULL_COMPRESSION":
                firstStateStageStatus = true;
                secondStateStageStatus = true;
            break;
            case "NO_COMPRESSION":
                firstStateStageStatus = false;
                secondStateStageStatus = false;
            break;

        }
        
        firstStage.set(firstStateStageStatus);
        secondStage.set(secondStateStageStatus);
        
    }

    public boolean[] getSolenoidStatuses(){
        boolean stageStatuses[] = new boolean[2];
        stageStatuses[0]= firstStage.get();
        stageStatuses[1]= firstStage.get();

        return stageStatuses;

    }


    
       






}
