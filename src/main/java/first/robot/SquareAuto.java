package first.robot;

import org.wpilib.opmode.Autonomous;
import org.wpilib.opmode.PeriodicOpMode;

@Autonomous
public class SquareAuto extends PeriodicOpMode{
    private final Robot robot;
    private final int SQUARE_SIZE = 200; // ticks
    private final int TOLERANCE = 10; //ticks
    private State state = State.FORWARD;
    boolean hasRun = false;

    enum State{
        FORWARD,
        LEFT,
        BACKWARD,
        RIGHT,
        DONE

    }

    public SquareAuto(Robot robot) {
        this.robot = robot;
        robot.initTelopDriving();
        state = State.FORWARD;
    }

    @Override
    public void periodic() {
        switch (state) {
            case FORWARD:
                robot.setSetpoints(SQUARE_SIZE, 0);
                if(robot.isDoneMoving(TOLERANCE)){
                    state = State.LEFT;
                }
                break;
            case LEFT:
                robot.setSetpoints(SQUARE_SIZE, SQUARE_SIZE);
                if(robot.isDoneMoving(TOLERANCE)){
                    state = State.BACKWARD;
                }
                break;
            case BACKWARD:
                robot.setSetpoints(0, SQUARE_SIZE);
                if(robot.isDoneMoving(TOLERANCE)){
                    state = State.RIGHT;
                }
                break;
            case RIGHT:
                robot.setSetpoints(0, 0);
                if (robot.isDoneMoving(TOLERANCE)) {
                    state = State.DONE;
                }
                break;
            default:
                break;
        }
        
    }

    @Override
    public void end() {
        //i only have this because for some reaosn periodic seems to run beofore start so i need to reset the state to forward. 
        state = State.FORWARD;
    }

    @Override
    public void start() {
        state = State.FORWARD;
        robot.resetEncoders();
        robot.setPidConstants();
    }
}
