package first.robot;

import org.wpilib.driverstation.Gamepad;
import org.wpilib.opmode.PeriodicOpMode;
import org.wpilib.opmode.Teleop;

@Teleop
public class MecanumTeleop extends PeriodicOpMode {
  private final Robot robot;
  private final Gamepad gamepad = new Gamepad(0);
  private final float MAX_SPEED = 0.5f;

  /** The Robot instance is passed into the opmode via the constructor. */
  public MecanumTeleop(Robot robot) {
    this.robot = robot;
  }

  @Override
  public void periodic() {
    robot.setDrivePowers(
      -gamepad.getLeftY() * MAX_SPEED,
      gamepad.getLeftX() * MAX_SPEED,
      gamepad.getRightX() * MAX_SPEED
    );
  }
}