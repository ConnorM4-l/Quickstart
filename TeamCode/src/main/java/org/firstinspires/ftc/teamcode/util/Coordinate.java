package org.firstinspires.ftc.teamcode.util;

public class Coordinate {
    private Double x;
    private Double y;

    public Coordinate()
    {
        x = 0.0;
        y = 0.0;
    }
    public  Coordinate( Double i,Double j)
    {
        x = i;
        y = j;
    }
    public Double getX() { return x;}
    public Double getY() { return y;}

    public void incermentX(double val)
    {
        x += val;
    }
    public void incermentY(double val)
    {
        y += val;
    }
    public Double getDistance ( Coordinate other)
    {
        Double displX = x - other.getX();
        Double displY = y - other.getY();
        return Math.sqrt(displX * displX + displY * displY);
    }
    public Double getAngle ( Coordinate other, Double facingAngle)
    {
        Double displX = other.getX() - x;
        Double displY = other.getY() - y;
        Double ret = Math.atan2(displY,displX) - facingAngle;
        return ((ret >= 0) ? ret : Math.PI*2 + ret) % Math.PI*2;
    }


}
