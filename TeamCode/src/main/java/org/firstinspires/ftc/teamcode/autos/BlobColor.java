package org.firstinspires.ftc.teamcode.autos;

import org.firstinspires.ftc.teamcode.vision2.blobvision.BlobConfig;

/**
 * A kind of artifact the camera can look for.
 *
 * <ul>
 *   <li><b>pipeline</b> - Limelight color pipeline index tuned for this color (color-blob mode).</li>
 *   <li><b>detectorClass</b> - class label in your neural model's labels.txt (neural mode).</li>
 * </ul>
 *
 * Edit the presets below (or make your own) so the indices match your Limelight.
 */
public final class BlobColor {

    // Neural mode (your model): pipeline index is unused, the label must match your labels.txt.
    // Matching is case-insensitive and also accepts the short name as a substring.
    public static final BlobColor POLLEN = new BlobColor("pollen", -1, "yellow_pollen");
    public static final BlobColor RED_NECTAR = new BlobColor("red_nectar", -1, "red_nectar");
    public static final BlobColor BLUE_NECTAR = new BlobColor("blue_nectar", -1, "blue_nectar");

    // Color-pipeline mode (no model): one Limelight color pipeline per color. Edit indices to match.
    public static final BlobColor PURPLE = new BlobColor("purple", 1, "purple");
    public static final BlobColor GREEN = new BlobColor("green", 2, "green");

    /** Sentinel: track every color listed in {@link BlobConfig#colors}. */
    public static final BlobColor ANY = new BlobColor("any", -1, "any");

    public final String name;
    public final int pipeline;
    public final String detectorClass;

    public BlobColor(String name, int pipeline, String detectorClass) {
        this.name = name;
        this.pipeline = pipeline;
        this.detectorClass = detectorClass;
    }

    public boolean isAny() {
        return this == ANY;
    }

    /** True if a detector class label belongs to this color (case-insensitive, substring ok). */
    public boolean matchesClass(String label) {
        if (label == null) return false;
        String l = label.toLowerCase();
        return l.equals(detectorClass.toLowerCase()) || l.contains(name.toLowerCase());
    }

    @Override
    public String toString() {
        return name;
    }
}
