package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.teamcode.maths.CubicPath;

public class PathList {

    public static Vector2d Temp = new Vector2d(0,0);

    public static CubicPath justPID(Pose2d pose) {
        Vector2d vec = new Vector2d(pose.getX(), pose.getY());
        Vector2d[] list = new Vector2d[]{vec, vec, vec, vec, vec, vec, vec, vec};
        return new CubicPath(list);
    }

    public static final Vector2d[] RedRightPathToSpikePoints = {
            new Vector2d(12,-60),
            new Vector2d(12,-57),
            new Vector2d(12,-52),
            new Vector2d(12,-47),
            new Vector2d(12,-37),
            new Vector2d(12,-36),
            new Vector2d(12,-35.65),
            new Vector2d(12,-35.5)
    };
    public static final CubicPath RedRightPathToSpike = new CubicPath(RedRightPathToSpikePoints);

    public static final Vector2d[] RedRightSpikeToStackPoints = {
            RedRightPathToSpikePoints[7],
            new Vector2d(12.5,-47),
            new Vector2d(37.3,-45.8),
            new Vector2d(34.3,-26.6),
            new Vector2d(-8.1,-13.6),
            new Vector2d(-22,-15),
            new Vector2d(-45.8,-16.3),
            new Vector2d(-60,-12)
    };
    public static final CubicPath RedRightSpikeToStack = new CubicPath(RedRightSpikeToStackPoints);

    public static final Vector2d[] RedStackToBoardPoints = {
            RedRightSpikeToStackPoints[7],
            new Vector2d(-40.8,-13),
            new Vector2d(-1,-14),
            new Vector2d(20.2,-14.6),
            new Vector2d(40.96,-25.1),
            new Vector2d(44,-30),
            new Vector2d(45.4,-36.6),
            new Vector2d(49,-36)
    };
    public static final CubicPath RedStackToBoard = new CubicPath(RedStackToBoardPoints);

    public static final Vector2d[] UShapePoints = {
        new Vector2d(-45,-60),
        new Vector2d(20,-60),
        new Vector2d(25,-60),
        new Vector2d(25,-45),
        new Vector2d(25, -30),
        new Vector2d(25,-22),
        new Vector2d(20,-12),
        new Vector2d(-45,-12)
    };
    public static final CubicPath UShapePath = new CubicPath(UShapePoints);

}
