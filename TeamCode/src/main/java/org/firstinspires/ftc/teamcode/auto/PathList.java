package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.maths.CubicPath;

public class PathList {

    public static Vector2d Temp = new Vector2d(0,0);

    public static final CubicPath RedLeftPathToSpike = new CubicPath(
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
            new Vector2d(-45,-15.5));
}
