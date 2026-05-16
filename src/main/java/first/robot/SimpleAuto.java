package first.robot;

import org.wpilib.opmode.Autonomous;
import org.wpilib.opmode.PeriodicOpMode;

@Autonomous
public class SimpleAuto extends PeriodicOpMode {
    private final Robot robot;
    private final int MOVMENT_SIZE = 200; // ticks

    public SimpleAuto(Robot robot) {
        this.robot = robot;
    }



    @Override
    public void periodic() {
        robot.updateDash();
    }

    @Override
    public void start() {
        robot.resetEncoders();
        robot.setPidConstants();
        robot.setSetpoints(MOVMENT_SIZE, 0);
    }
}
