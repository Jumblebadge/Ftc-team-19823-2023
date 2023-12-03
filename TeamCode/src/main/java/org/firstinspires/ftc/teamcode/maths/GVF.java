package org.firstinspires.ftc.teamcode.maths;

import com.acmerobotics.roadrunner.geometry.Vector2d;

public class GVF {

    CubicPath path;
    Vector2d R, closestPoint;
    double Kn;

    public GVF (CubicPath path, double Kn) {
        this.path = path;
        this.Kn = Kn;
    }

    public double calculateError() {
        return R.distTo(new Vector2d(0,0));
    }

    public void calculateEverything(Vector2d Robot) {
        closestPoint = path.findClosestPointOnPath(Robot);
        R = new Vector2d(Robot.getX() - closestPoint.getX(), Robot.getY() - closestPoint.getY());
    }

    public Vector2d output(Vector2d Robot) {
        calculateEverything(Robot);
        Vector2d tangent = path.getNormalizedTangent(path.guessT);
        Vector2d normal = path.getNormalizedNormal(path.guessT);
        double error = calculateError();

        //all that work for one line LMAO
        return tangent.minus(normal.times(Kn).times(error));
    }

}
