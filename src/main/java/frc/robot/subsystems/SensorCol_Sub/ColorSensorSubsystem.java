package frc.robot.subsystems.SensorCol_Sub;

import com.revrobotics.ColorSensorV3;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorMatch;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ColorSensorSubsystem extends SubsystemBase {

    private final I2C.Port i2cPort = I2C.Port.kOnboard;
    private final ColorSensorV3 colorSensor = new ColorSensorV3(i2cPort);
    private final ColorMatch colorMatch = new ColorMatch();
    

    private final Color kBlueTarget = new Color(0.143, 0.427, 0.429);
    private final Color kGreenTarget = new Color(0.197, 0.561, 0.240);
    private final Color kRedTarget = new Color(0.561, 0.232, 0.114);
    private final Color kYellowTarget = new Color(0.361, 0.524, 0.113);
    private final Color kOrangeTarget = new Color(0.526, 0.387, 0.087);

    public String colorString;

    public Color getColor(){
        Color detectedColor = colorSensor.getColor();
        ColorMatchResult matchResult = colorMatch.matchClosestColor(detectedColor);
        return matchResult.color;
    }

    public double getRed(){
        Color detectedColor = colorSensor.getColor();
        return detectedColor.red;
    }
    

    @Override
    public void periodic () {
        colorMatch.addColorMatch(kBlueTarget);
        colorMatch.addColorMatch(kGreenTarget);
        colorMatch.addColorMatch(kRedTarget);
        colorMatch.addColorMatch(kYellowTarget);
        colorMatch.addColorMatch(kOrangeTarget);

        Color detectedColor = colorSensor.getColor();

        ColorMatchResult match = colorMatch.matchClosestColor(detectedColor);

        if (match.color == kBlueTarget) {
            colorString = "Blue";
        }
        else if (match.color == kGreenTarget) {
            colorString = "Green";
        }
        else if (match.color == kRedTarget) {
            colorString = "Red";
        }
        else if (match.color == kYellowTarget) {
            colorString = "Yellow";
        }
        // else if (match.color == kOrangeTarget) {
        //     colorString = "Orange (Color dona)";
        // }
        else {
            colorString = "Unknown";
        }

        SmartDashboard.putNumber("Blue", detectedColor.blue);
        SmartDashboard.putNumber("Green", detectedColor.green);
        SmartDashboard.putNumber("Red", detectedColor.red);
        SmartDashboard.putNumber("Confidence", match.confidence);
        SmartDashboard.putString("Detected Color", colorString);

    }
    
}