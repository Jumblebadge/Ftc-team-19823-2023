package org.firstinspires.ftc.teamcode.maths;

import com.acmerobotics.roadrunner.geometry.Vector2d;

public class Path {

    public final Bezier[] beziers;

    public Path(Bezier... beziers) { this.beziers = beziers; }

    public Vector2d getPoint(double T) {
        return beziers[(int) Math.floor(T)].getPoint(T);
    }

    //TODO finish lmao

}
