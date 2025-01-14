package frc.robot;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Superstructure extends SubsystemBase{
    private ScoringLevel selectedLevel;

    private final GenericEntry[] levelButtons = new GenericEntry[4];
    private final boolean[] levelBooleanArray = new boolean[4];
    private final ShuffleboardTab tab = Shuffleboard.getTab("selector");

    public Superstructure(){
        levelButtons[0] = tab.add("L1", false).withWidget(BuiltInWidgets.kToggleButton).getEntry();
        levelButtons[1] = tab.add("L2", false).withWidget(BuiltInWidgets.kToggleButton).getEntry();
        levelButtons[2] = tab.add("L3", false).withWidget(BuiltInWidgets.kToggleButton).getEntry();
        levelButtons[3] = tab.add("L4", false).withWidget(BuiltInWidgets.kToggleButton).getEntry();
        
    }
    public void selectLevel(ScoringLevel level){
        this.selectedLevel = level;
    }

    public ScoringLevel getLevel(){
        return selectedLevel;
    }





    public enum ScoringLevel{
        L1,
        L2, 
        L3, 
        L4
    }

    @Override
    public void periodic() {

        for (int i = 0; i < 4; i++) {
            if (levelButtons[i].getBoolean(false) != levelBooleanArray[i] & levelButtons[i].getBoolean(false)) {
                levelButtons[0].setBoolean(false);
                levelBooleanArray[0] = false;
                levelButtons[1].setBoolean(false);
                levelBooleanArray[1] = false;
                levelButtons[2].setBoolean(false);
                levelBooleanArray[2] = false;
                levelButtons[3].setBoolean(false);
                levelBooleanArray[3] = false;
                levelButtons[i].setBoolean(true);
                levelBooleanArray[i] = true;
            } else if (levelButtons[i].getBoolean(false) != levelBooleanArray[i]) {
                levelBooleanArray[i] = false;
            }
        }
        if(levelButtons[0].getBoolean(false)){
            selectedLevel = ScoringLevel.L1;
        }
        if(levelButtons[1].getBoolean(false)){
            selectedLevel = ScoringLevel.L2;
        }
        if(levelButtons[2].getBoolean(false)){
            selectedLevel = ScoringLevel.L3;
        }
        if(levelButtons[3].getBoolean(false)){
            selectedLevel = ScoringLevel.L4;
        }
        System.out.println("scoring level: " + selectedLevel);
        
    }
    
}
