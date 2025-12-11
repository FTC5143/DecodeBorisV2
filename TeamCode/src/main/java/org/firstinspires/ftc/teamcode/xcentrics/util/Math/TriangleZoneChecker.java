package org.firstinspires.ftc.teamcode.xcentrics.util.Math;


import com.pedropathing.geometry.Pose;

public class TriangleZoneChecker {

    // --------------------------------------------------------------
    // Simple 2D point class (kept internal — NOT RoadRunner)
    // --------------------------------------------------------------
    static class Vec2 {
        double x, y;

        Vec2(double x, double y) {
            this.x = x;
            this.y = y;
        }

        Vec2 sub(Vec2 o) {
            return new Vec2(this.x - o.x, this.y - o.y);
        }

        double dot(Vec2 o) {
            return this.x * o.x + this.y * o.y;
        }
    }

    // --------------------------------------------------------------
    // Stores one triangle (A, B, C)
    // --------------------------------------------------------------
    static class Triangle {
        Vec2 A, B, C;

        Triangle(Vec2 A, Vec2 B, Vec2 C) {
            this.A = A;
            this.B = B;
            this.C = C;
        }
    }

    // --------------------------------------------------------------
    // Barycentric point-in-triangle check
    // --------------------------------------------------------------
    static boolean pointInTriangle(Vec2 P, Triangle t) {

        Vec2 A = t.A;
        Vec2 B = t.B;
        Vec2 C = t.C;

        Vec2 v0 = C.sub(A);   // AC
        Vec2 v1 = B.sub(A);   // AB
        Vec2 v2 = P.sub(A);   // AP

        double dot00 = v0.dot(v0);
        double dot01 = v0.dot(v1);
        double dot02 = v0.dot(v2);
        double dot11 = v1.dot(v1);
        double dot12 = v1.dot(v2);

        double invDen = 1.0 / (dot00 * dot11 - dot01 * dot01);

        double u = (dot11 * dot02 - dot01 * dot12) * invDen;
        double v = (dot00 * dot12 - dot01 * dot02) * invDen;

        return (u >= 0) && (v >= 0) && (u + v <= 1);
    }

    // --------------------------------------------------------------
    // Triangles you want to check against
    // --------------------------------------------------------------
    Triangle bigTriangle;
    Triangle smallTriangle;

    public TriangleZoneChecker() {

        // TODO: Replace these with your real coordinates in field space (inches)
        bigTriangle = new Triangle(
                new Vec2(0, 0),
                new Vec2(72, 0),
                new Vec2(36, 72)
        );

        smallTriangle = new Triangle(
                new Vec2(20, 20),
                new Vec2(52, 20),
                new Vec2(36, 50)
        );
    }

    // --------------------------------------------------------------
    // TRUE if robot is inside big OR small triangle
    // --------------------------------------------------------------
    public boolean isInsideAnyTriangle(Pose pose) {

        // Use Pedro Pathing's pose.x and pose.y directly
        Vec2 p = new Vec2(pose.getX(), pose.getY());

        if (pointInTriangle(p, bigTriangle)) return true;
        if (pointInTriangle(p, smallTriangle)) return true;

        return false;
    }
}
