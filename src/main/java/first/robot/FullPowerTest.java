package first.robot;

import org.wpilib.driverstation.Gamepad;
import org.wpilib.math.util.MathUtil;
import org.wpilib.opmode.PeriodicOpMode;
import org.wpilib.opmode.Teleop;

@Teleop
public class FullPowerTest extends PeriodicOpMode {
  private final Robot robot;
  private final float DEADBAND = 0.1f;

  /** The Robot instance is passed into the opmode via the constructor. */
  public FullPowerTest(Robot robot) {
    this.robot = robot;
  }

  @Override
  public void periodic() {

    robot.setDrivePowers(
      1,
      0,
      0
    );
  }
  @Override
  public void start() {
  }
}