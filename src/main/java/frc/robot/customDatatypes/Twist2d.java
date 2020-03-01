package frc.robot.customDatatypes;

// analogue for team 254
// stores turn, x movement, y movement 

public class Twist2d{
    // how fast to travel in both directions
    public double dx;
    public double dy;

    // the angle
    public double dtheta;

    public Twist2d(double _dx, double _dy, double _dtheta){
        dx = _dx;
        dy = _dy;
        dtheta = _dtheta;
    }
}