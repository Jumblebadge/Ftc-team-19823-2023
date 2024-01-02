package org.firstinspires.ftc.teamcode.maths;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

public class GVF {

    CubicPath path;
    Vector2d R, closestPoint;
    FtcDashboard dashboard;
    double Kn;

    public GVF(FtcDashboard dashboard, CubicPath path, double Kn) {
        this.path = path;
        this.Kn = Kn;
        this.dashboard = dashboard;
    }

    public void setKn(double Kn) {
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
        double max = Math.max(tangent.minus(normal.times(Kn).times(error)).getX(), tangent.minus(normal.times(Kn).times(error)).getY());
        drawPath(dashboard, path, new Pose2d(Robot.getX(), Robot.getY()));
        if (max > 1) {
            return tangent.minus(normal.times(Kn).times(error)).div(max);
        }
        return tangent.minus(normal.times(Kn).times(error));
    }

    public void drawPath(FtcDashboard dash, CubicPath path, Pose2d robot) {
        TelemetryPacket packet = new TelemetryPacket();
        Canvas canvas = packet.fieldOverlay();
        Vector2d[] points = new Vector2d[30];
        double[] x = new double[points.length];
        double[] y = new double[points.length];
        for (int i = 0; i < points.length; i++) {
            points[i] = path.getPoint((double) i * 3 / points.length);
            x[i] = points[i].getX();
            y[i] = points[i].getY();
        }
        canvas.strokePolyline(x,y);
        drawRobot(canvas, robot);
        drawRobot(canvas, new Pose2d(closestPoint.getX(), closestPoint.getY()));
        dash.sendTelemetryPacket(packet);
    }

    public static void drawRobot(Canvas canvas, Pose2d pose) {
        canvas.strokeCircle(pose.getX(), pose.getY(), 9);
        Vector2d v = pose.headingVec().times(9);
        double x1 = pose.getX() + v.getX() / 2, y1 = pose.getY() + v.getY() / 2;
        double x2 = pose.getX() + v.getX(), y2 = pose.getY() + v.getY();
        canvas.strokeLine(x1, y1, x2, y2);
    }

}
