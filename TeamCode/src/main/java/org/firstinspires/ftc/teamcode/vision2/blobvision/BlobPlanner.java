package org.firstinspires.ftc.teamcode.vision2.blobvision;

import java.util.ArrayList;
import java.util.List;

/**
 * Weaver-style target ordering for the blobs the camera currently sees.
 *
 * - picks the best {@code maxTargets} and the cheapest visiting order (length + turning cost)
 * - drops targets that sit inside, or are straight-line blocked by, circular obstacles
 * - exact search up to {@code bruteForceLimit} candidates, greedy beyond that
 *
 * With a pose it plans in the field frame (obstacles are field coordinates).
 * With a null pose it plans in the robot frame (obstacles are ignored).
 */
public final class BlobPlanner {

    public static final class Circle {
        public final double x, y, r;
        public Circle(double x, double y, double r) { this.x = x; this.y = y; this.r = r; }
    }

    public static final class Result {
        /** Chosen targets in visiting order. */
        public final List<Blob> order;
        /** Targets that were blocked, un-rangeable, or beyond maxTargets. */
        public final List<Blob> skipped;
        public final double lengthIn;

        Result(List<Blob> order, List<Blob> skipped, double lengthIn) {
            this.order = order;
            this.skipped = skipped;
            this.lengthIn = lengthIn;
        }
    }

    private final List<Circle> obstacles = new ArrayList<>();
    private int maxTargets = Integer.MAX_VALUE;
    private boolean ordered = false;
    private double turnCostWeight = 10.0;   // inches of "cost" per radian of turning
    private double clearanceIn = 4.0;
    private double robotRadiusIn = 9.0;
    private int bruteForceLimit = 8;
    private boolean excludeBlocked = true;

    // search scratch
    private double bestCost;
    private List<Blob> bestSeq;

    // ------------------------------------------------------------------ builder

    public BlobPlanner obstacle(double x, double y, double radius) { obstacles.add(new Circle(x, y, radius)); return this; }
    public BlobPlanner maxTargets(int n) { maxTargets = n; return this; }
    /** Keep the camera's order instead of optimizing. */
    public BlobPlanner ordered(boolean v) { ordered = v; return this; }
    public BlobPlanner turnCostWeight(double v) { turnCostWeight = v; return this; }
    public BlobPlanner clearanceIn(double v) { clearanceIn = v; return this; }
    public BlobPlanner robotRadiusIn(double v) { robotRadiusIn = v; return this; }
    public BlobPlanner bruteForceLimit(int v) { bruteForceLimit = v; return this; }
    public BlobPlanner excludeBlocked(boolean v) { excludeBlocked = v; return this; }

    // ------------------------------------------------------------------ planning

    public Result plan(RobotPose start, List<Blob> targets) {
        boolean field = start != null;
        double sx = field ? start.x : 0;
        double sy = field ? start.y : 0;
        double sh = field ? start.heading : 0;

        List<Blob> cand = new ArrayList<>();
        List<Blob> skipped = new ArrayList<>();
        for (Blob b : targets) {
            boolean usable = field ? b.hasField() : b.hasRange();
            if (!usable) { skipped.add(b); continue; }
            if (field && excludeBlocked && blocked(sx, sy, b.planX(true), b.planY(true))) {
                skipped.add(b);
                continue;
            }
            cand.add(b);
        }

        int n = cand.size();
        int k = Math.min(maxTargets, n);
        List<Blob> order;
        if (k <= 0) {
            order = new ArrayList<>();
        } else if (ordered) {
            order = new ArrayList<>(cand.subList(0, k));
        } else if (n <= bruteForceLimit) {
            bestCost = Double.MAX_VALUE;
            bestSeq = null;
            dfs(field, cand, new boolean[n], new ArrayList<Blob>(), sx, sy, sh, 0, k);
            order = bestSeq != null ? bestSeq : new ArrayList<Blob>();
        } else {
            order = greedy(field, cand, sx, sy, sh, k);
        }

        for (Blob b : cand) if (!order.contains(b)) skipped.add(b);
        return new Result(order, skipped, length(field, order, sx, sy));
    }

    private void dfs(boolean field, List<Blob> cand, boolean[] used, List<Blob> seq,
                     double x, double y, double h, double cost, int k) {
        if (cost >= bestCost) return;
        if (seq.size() == k) {
            bestCost = cost;
            bestSeq = new ArrayList<>(seq);
            return;
        }
        for (int i = 0; i < cand.size(); i++) {
            if (used[i]) continue;
            Blob b = cand.get(i);
            double bx = b.planX(field), by = b.planY(field);
            double step = stepCost(field, x, y, h, bx, by);
            used[i] = true;
            seq.add(b);
            dfs(field, cand, used, seq, bx, by, Math.atan2(by - y, bx - x), cost + step, k);
            seq.remove(seq.size() - 1);
            used[i] = false;
        }
    }

    private List<Blob> greedy(boolean field, List<Blob> cand, double x, double y, double h, int k) {
        List<Blob> left = new ArrayList<>(cand);
        List<Blob> out = new ArrayList<>();
        while (out.size() < k && !left.isEmpty()) {
            Blob best = null;
            double bestStep = Double.MAX_VALUE;
            for (Blob b : left) {
                double s = stepCost(field, x, y, h, b.planX(field), b.planY(field));
                if (s < bestStep) { bestStep = s; best = b; }
            }
            double bx = best.planX(field), by = best.planY(field);
            h = Math.atan2(by - y, bx - x);
            x = bx;
            y = by;
            out.add(best);
            left.remove(best);
        }
        return out;
    }

    private double stepCost(boolean field, double x, double y, double h, double bx, double by) {
        double dx = bx - x, dy = by - y;
        double len = Math.hypot(dx, dy);
        double turn = Math.abs(BlobMath.angleWrap(Math.atan2(dy, dx) - h));
        double c = len + turnCostWeight * turn;
        if (field && crosses(x, y, bx, by)) c += 10000;
        return c;
    }

    private double length(boolean field, List<Blob> order, double x, double y) {
        double total = 0;
        for (Blob b : order) {
            double bx = b.planX(field), by = b.planY(field);
            total += Math.hypot(bx - x, by - y);
            x = bx;
            y = by;
        }
        return total;
    }

    // ------------------------------------------------------------------ obstacle geometry

    private boolean blocked(double x1, double y1, double x2, double y2) {
        return crosses(x1, y1, x2, y2);
    }

    private boolean crosses(double x1, double y1, double x2, double y2) {
        for (Circle c : obstacles) {
            if (distToSegment(c.x, c.y, x1, y1, x2, y2) < c.r + clearanceIn + robotRadiusIn) return true;
        }
        return false;
    }

    private static double distToSegment(double px, double py, double x1, double y1, double x2, double y2) {
        double dx = x2 - x1, dy = y2 - y1;
        double len2 = dx * dx + dy * dy;
        if (len2 < 1e-9) return Math.hypot(px - x1, py - y1);
        double t = BlobMath.clamp(((px - x1) * dx + (py - y1) * dy) / len2, 0, 1);
        return Math.hypot(px - (x1 + t * dx), py - (y1 + t * dy));
    }
}
