package first.robot;

import org.wpilib.framework.OpModeRobot;
import org.wpilib.hardware.hal.CANBusMap;
import org.wpilib.opmode.PeriodicOpMode;
import org.wpilib.opmode.Utility;

import com.revrobotics.spark.A301;

@Utility
public class A301Test extends PeriodicOpMode {

    private final OpModeRobot robot;

    private A301 motor = new A301(CANBusMap.CAN_D0);
    // private A301 motor1 = new A301(CANBusMap.CAN_D5);


    public A301Test(OpModeRobot robot) {
        this.robot = robot;
    }

    @Override
    public void periodic() {
        motor.setVoltage(0);
        // motor1.setThrottle(1);
    }

}
