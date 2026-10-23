package org.firstinspires.ftc.teamcode.vision2.blobvision;

/**
 * Stateless control helpers. Copy any of these straight into a TeleOp or Auto.
 * Command convention: forward +, left +, turn CCW+ (a target on the left gives a positive turn).
 */
public final class BlobControl {

    private BlobControl() {}

    public static final class Command {
        public static final Command ZERO = new Command(0, 0, 0);
        public final double forward, left, turn;
        public Command(double forward, double left, double turn) {
            this.forward = forward;
            this.left = left;
            this.turn = turn;
        }
    }

    /** Turn power that points the robot at the blob. Great for TeleOp auto-aim. */
    public static double turnToward(BlobConfig c, Blob b) {
        double err = b.bearingDeg;
        if (Math.abs(err) < c.aimToleranceDeg) return 0;
        double t = c.kTurn * err;
        t = Math.signum(t) * Math.max(Math.abs(t), c.minTurn);
        return BlobMath.clamp(t, -c.maxTurn, c.maxTurn);
    }

    /** Forward power that closes the distance, faded out while badly misaligned. */
    public static double forwardToward(BlobConfig c, Blob b) {
        double f;
        if (b.hasRange()) {
            double remaining = b.distanceIn - c.engageDistanceIn;
            if (remaining <= 0) return 0;
            f = BlobMath.clamp(c.kForward * remaining, c.minForward, c.maxForward);
        } else {
            f = c.minForward * 1.5; // no range: creep while aiming
        }
        double align = BlobMath.clamp(1.0 - Math.abs(b.bearingDeg) / c.alignSlowDeg, 0, 1);
        return f * align;
    }

    /** Full visual-servo chase command toward a blob. */
    public static Command chase(BlobConfig c, Blob b) {
        return new Command(forwardToward(c, b), 0, turnToward(c, b));
    }

    /** Blind straight push into the intake. */
    public static Command push(BlobConfig c) {
        return new Command(c.pushPower, 0, 0);
    }

    /** Spin-sweep that reverses every searchFlipMs. startDir = +1 (CCW) or -1 (CW). */
    public static Command search(BlobConfig c, long msInSearch, int startDir) {
        long flips = c.searchFlipMs <= 0 ? 0 : msInSearch / c.searchFlipMs;
        int dir = (flips % 2 == 0) ? startDir : -startDir;
        return new Command(0, 0, dir * c.searchTurnPower);
    }
}
