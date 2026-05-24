package first.robot;

import org.wpilib.hardware.bus.I2C.Port;
import org.wpilib.opmode.PeriodicOpMode;
import org.wpilib.opmode.Utility;
import org.wpilib.smartdashboard.SmartDashboard;
import org.wpilib.util.Color;

import com.revrobotics.ColorSensorV3;

@Utility
public class ColorSensorTest extends PeriodicOpMode {

    private final Robot robot;

    private final ColorSensorV3 colorV3 = new ColorSensorV3(Port.PORT_0);



    public ColorSensorTest(Robot robot)
    {
        this.robot = robot;
    }

    @Override
    public void periodic() {
        Color color = colorV3.getColor();
        SmartDashboard.putNumber("red", color.red);
        SmartDashboard.putNumber("green", color.green);
        SmartDashboard.putNumber("blue", color.blue);
        SmartDashboard.putNumber("proximty", colorV3.getProximity());

    }
    
}
