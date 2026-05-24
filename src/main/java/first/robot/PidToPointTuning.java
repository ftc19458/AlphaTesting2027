package first.robot;

import static org.wpilib.units.Units.Inches;

import org.wpilib.math.geometry.Pose2d;
import org.wpilib.opmode.PeriodicOpMode;
import org.wpilib.opmode.Utility;
import org.wpilib.smartdashboard.SmartDashboard;

@Utility
public class PidToPointTuning extends PeriodicOpMode {

    private Robot robot;

    public PidToPointTuning(Robot robot){
        this.robot = robot;
    }

    @Override
    public void start() {
        robot.resetPinpoint();
        SmartDashboard.putNumber("xTarget", 0);
        SmartDashboard.putNumber("yTarget", 0);
        SmartDashboard.putNumber("hTarget", 180);
    }

    @Override
    public void periodic() {

        robot.updatePinpont();

        double xTarget = SmartDashboard.getNumber("xTarget", 0);
        double yTarget = SmartDashboard.getNumber("yTarget", 0);
        double hTarget = SmartDashboard.getNumber("hTarget", 0);

        robot.setSetpoint(xTarget,yTarget,hTarget);

        robot.updatePowers();

        Pose2d pos = robot.getCurPose2d();

        SmartDashboard.putNumber("xPosition", pos.getMeasureX().in(Inches));
        SmartDashboard.putNumber("yPosition", pos.getMeasureY().in(Inches));
        SmartDashboard.putNumber("hPosition", pos.getRotation().getDegrees());
    }
    
}
