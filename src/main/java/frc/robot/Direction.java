package frc.robot;

public enum Direction {
    North(0),
    East(90),
    South(180),
    West(-90);

    public final double value;

    Direction(double degrees) {
        value = degrees;
    }
}
