package first.robot.sensors;


import static org.wpilib.units.Units.Meters;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;

import org.wpilib.hardware.bus.I2C;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.units.measure.Distance;

public class GoBildaPinpoint {
    
    private static final int DEFAULT_ADDRESS = 0x31;

    private final I2C i2c;

    private int deviceStatus;
    private int loopTime;

    private int xEncoderValue;
    private int yEncoderValue;

    private float xPosition;
    private float yPosition;
    private float heading;

    private float xVelocity;
    private float yVelocity;
    private float headingVelocity;

    public enum EncoderDirection {
        FORWARD,
        REVERSED
    }

    private enum Register {
        DEVICE_ID(1),
        DEVICE_VERSION(2),
        DEVICE_STATUS(3),
        DEVICE_CONTROL(4),
        LOOP_TIME(5),
        X_ENCODER_VALUE(6),
        Y_ENCODER_VALUE(7),
        X_POSITION(8),
        Y_POSITION(9),
        H_ORIENTATION(10),
        X_VELOCITY(11),
        Y_VELOCITY(12),
        H_VELOCITY(13),
        MM_PER_TICK(14),
        X_POD_OFFSET(15),
        Y_POD_OFFSET(16),
        YAW_SCALAR(17),
        BULK_READ(18);

        public final int value;

        Register(int value) {
            this.value = value;
        }
    }

    public GoBildaPinpoint(I2C.Port port) {
        i2c = new I2C(port, DEFAULT_ADDRESS);
    }

    public void update() {

        byte[] buffer = new byte[40];

        boolean failed = i2c.read(Register.BULK_READ.value, 40, buffer);

        if (failed) {
            return;
        }

        ByteBuffer bb = ByteBuffer.wrap(buffer);
        bb.order(ByteOrder.LITTLE_ENDIAN);

        deviceStatus = bb.getInt();
        loopTime = bb.getInt();

        xEncoderValue = bb.getInt();
        yEncoderValue = bb.getInt();

        xPosition = bb.getFloat();
        yPosition = bb.getFloat();
        heading = bb.getFloat();

        xVelocity = bb.getFloat();
        yVelocity = bb.getFloat();
        headingVelocity = bb.getFloat();
    }

    private void writeInt(Register reg, int value) {
        byte[] data = ByteBuffer.allocate(4)
                .order(ByteOrder.LITTLE_ENDIAN)
                .putInt(value)
                .array();

        byte[] out = new byte[5];
        out[0] = (byte) reg.value;

        System.arraycopy(data, 0, out, 1, 4);

        i2c.writeBulk(out);
    }

    private void writeFloat(Register reg, float value) {

        byte[] data = ByteBuffer.allocate(4)
                .order(ByteOrder.LITTLE_ENDIAN)
                .putFloat(value)
                .array();

        byte[] out = new byte[5];
        out[0] = (byte) reg.value;

        System.arraycopy(data, 0, out, 1, 4);

        i2c.writeBulk(out);
    }

    public void resetPosAndIMU() {
        writeInt(Register.DEVICE_CONTROL, 1 << 1);
    }

    public void recalibrateIMU() {
        writeInt(Register.DEVICE_CONTROL, 1 << 0);
    }

    public void setEncoderDirections(
            EncoderDirection x,
            EncoderDirection y
    ) {

        if (x == EncoderDirection.FORWARD) {
            writeInt(Register.DEVICE_CONTROL, 1 << 5);
        } else {
            writeInt(Register.DEVICE_CONTROL, 1 << 4);
        }

        if (y == EncoderDirection.FORWARD) {
            writeInt(Register.DEVICE_CONTROL, 1 << 3);
        } else {
            writeInt(Register.DEVICE_CONTROL, 1 << 2);
        }
    }

    public void setOffsetsMM(double xOffsetMM, double yOffsetMM) {
        writeFloat(Register.X_POD_OFFSET, (float) xOffsetMM);
        writeFloat(Register.Y_POD_OFFSET, (float) yOffsetMM);
    }

    public void setEncoderResolution(double ticksPerMM) {
        writeFloat(Register.MM_PER_TICK, (float) ticksPerMM);
    }

    public Pose2d getPoseMeters() {

        return new Pose2d(
                xPosition / 1000.0,
                yPosition / 1000.0,
                Rotation2d.fromRadians(heading)
        );
    }

    public double getXMeters() {
        return xPosition / 1000.0;
    }

    public double getYMeters() {
        return yPosition / 1000.0;
    }

    public Rotation2d getHeading() {
        return Rotation2d.fromRadians(heading);
    }

    public double getHeadingRadians() {
        return heading;
    }

    public double getXVelocityMetersPerSec() {
        return xVelocity / 1000.0;
    }

    public double getYVelocityMetersPerSec() {
        return yVelocity / 1000.0;
    }

    public double getHeadingVelocityRadPerSec() {
        return headingVelocity;
    }

    public int getEncoderX() {
        return xEncoderValue;
    }

    public int getEncoderY() {
        return yEncoderValue;
    }

    public int getLoopTimeMicros() {
        return loopTime;
    }

    public int getDeviceStatus() {
        return deviceStatus;
    }

    public Pose2d getPose() {
        Distance xDist = Distance.ofRelativeUnits(getXMeters(), Meters);
        Distance yDist = Distance.ofRelativeUnits(getYMeters(), Meters);
        Rotation2d rot = Rotation2d.fromRadians(getHeadingRadians());
        return new Pose2d(xDist,yDist,rot);

    }
}