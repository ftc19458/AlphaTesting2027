package first.robot;

import org.wpilib.hardware.bus.I2C.Port;
import org.wpilib.opmode.PeriodicOpMode;
import org.wpilib.opmode.Utility;
import org.wpilib.smartdashboard.SmartDashboard;


import first.robot.sensors.GoBildaPinpoint;
import first.robot.sensors.GoBildaPinpoint.EncoderDirection;

@Utility
public class PinpointTest extends PeriodicOpMode {

    private final Robot robot;

    private final GoBildaPinpoint pinpoint = new GoBildaPinpoint(Port.PORT_1);



    public PinpointTest(Robot robot)
    {
        this.robot = robot;
        pinpoint.setEncoderDirections(EncoderDirection.FORWARD, EncoderDirection.FORWARD);
        pinpoint.setEncoderResolution(19.8943678865);
        pinpoint.setOffsetsMM(139.7,-25.4);
    }

    @Override
    public void periodic() {
        pinpoint.update();
        SmartDashboard.putNumber("x", pinpoint.getXMeters() * 39.37007874);
        SmartDashboard.putNumber("y", pinpoint.getYMeters() * 39.37007874);
        SmartDashboard.putNumber("heading", pinpoint.getHeadingRadians() * (180/ Math.PI));

    }

    @Override
    public void start() {
        pinpoint.resetPosAndIMU();
    }
    
}
