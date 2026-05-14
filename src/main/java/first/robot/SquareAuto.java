package first.robot;

import org.wpilib.opmode.Autonomous;
import org.wpilib.opmode.PeriodicOpMode;
import org.wpilib.smartdashboard.SmartDashboard;

@Autonomous
public class SquareAuto extends PeriodicOpMode {
    private final Robot robot;
    private final int SQUARE_SIZE = 100; // ticks
    private final int ENCODER_TOL = 10; // ticks

    public SquareAuto(Robot robot) {
        this.robot = robot;
    }

    private enum State {
        FORWARD,
        STRAFE_RIGHT,
        BACKWARD,
        STRAFE_LEFT,
        DONE
    }

    private State currentState = State.FORWARD;

    @Override
    public void periodic() {
        SmartDashboard.putString("current state", currentState.toString());
        switch (currentState) {
            case FORWARD:
                robot.setDrivePowers(SQUARE_SIZE, 0, 0);
                currentState = State.STRAFE_RIGHT;
                break;
            case STRAFE_RIGHT:
                if (robot.isDoneMoving(SQUARE_SIZE, ENCODER_TOL)) {
                    robot.resetEncoders();
                    robot.setDrivePowers(0, SQUARE_SIZE, 0);
                    currentState = State.BACKWARD;
                }
                break;
            case BACKWARD:
                if (robot.isDoneMoving(SQUARE_SIZE, ENCODER_TOL)) {
                    robot.resetEncoders();
                    robot.setDrivePowers(-SQUARE_SIZE, 0, 0);
                    currentState = State.STRAFE_LEFT;
                }
                break;
            case STRAFE_LEFT:
                if (robot.isDoneMoving(SQUARE_SIZE, ENCODER_TOL)) {
                    robot.resetEncoders();
                    robot.setDrivePowers(0, -SQUARE_SIZE, 0);
                    currentState = State.DONE;
                }
                break;
            case DONE:
            default:
                robot.setDrivePowers(0, 0, 0);
                break;
            }
    }

    @Override
    public void start() {
        robot.resetEncoders();
        currentState = State.FORWARD;
    }
}
