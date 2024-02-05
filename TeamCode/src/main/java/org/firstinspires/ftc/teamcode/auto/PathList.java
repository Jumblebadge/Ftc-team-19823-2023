package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.teamcode.maths.CubicPath;

public class PathList {

    public static Vector2d Temp = new Vector2d(0,0);

    public static CubicPath justPID(Pose2d pose) {
        Vector2d vec = new Vector2d(pose.getX(), pose.getY());
        Vector2d[] list = new Vector2d[]{vec, vec, vec, vec, vec, vec, vec, vec, vec, vec, vec, vec};
        return new CubicPath(list);
    }

    public static final Vector2d[] RedLeftPathToSpikePoints = {
            new Vector2d(-45,-60),
            new Vector2d(-45,-59),
            new Vector2d(-45,-57),
            new Vector2d(-45,-52),
            new Vector2d(-45,-47),
            new Vector2d(-45,-41),
            new Vector2d(-45,-39),
            new Vector2d(-45,-37),
            new Vector2d(-45,-36),
            new Vector2d(-45,-35.9),
            new Vector2d(-45,-35.65),
            new Vector2d(-45,-35.5)
    };
    public static final CubicPath RedLeftPathToSpike = new CubicPath(RedLeftPathToSpikePoints);

    public static final Vector2d[] RedRightPathToSpikePoints = {
            new Vector2d(3.5,-60),
            new Vector2d(3.5,-59),
            new Vector2d(3.5,-57),
            new Vector2d(3.5,-52),
            new Vector2d(3.5,-47),
            new Vector2d(3.5,-41),
            new Vector2d(3.5,-39),
            new Vector2d(3.5,-37),
            new Vector2d(3.5,-36),
            new Vector2d(3.5,-35.9),
            new Vector2d(3.5,-35.65),
            new Vector2d(3.5,-35.5)
    };
    public static final CubicPath RedRightPathToSpike = new CubicPath(RedRightPathToSpikePoints);

    public static final Vector2d[] RedRightSpikeToStackPoints = {
            RedRightPathToSpikePoints[11],
            new Vector2d(3.2,-55.6),
            new Vector2d(25,-57.5),
            new Vector2d(25,-62.2),
            new Vector2d(25,-36.2),
            new Vector2d(25,-36.2),
            new Vector2d(-5.6,-10),
            new Vector2d(-21,-17),
            new Vector2d(-21,-17),
            new Vector2d(-37,-17),
            new Vector2d(-46,-19),
            new Vector2d(-63.7,-12.4)
    };
    public static final CubicPath RedRightSpikeToStack = new CubicPath(RedRightSpikeToStackPoints);

    public static final Vector2d[] UShapePoints = {
        new Vector2d(-45,-60),
        new Vector2d(20,-60),
        new Vector2d(25,-60),
        new Vector2d(25,-45),
        new Vector2d(25,-45),
        new Vector2d(25,-30),
        new Vector2d(25, -30),
        new Vector2d(25,-22),
        new Vector2d(25,-22),
        new Vector2d(25,-14),
        new Vector2d(20,-12),
        new Vector2d(-45,-12)
    };
    public static final CubicPath UShapePath = new CubicPath(UShapePoints);

}
