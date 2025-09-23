package com.airbrakesplugin.util;

import java.io.BufferedReader;
import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.*;

/**
 * GenericFunction2D — simplified, dependency-free 2D surface for ΔDrag (N).
 *
 * - Strictly loads columns: Mach, DeploymentPercentage, DeltaDrag_N (case-insensitive).
 * - Assumes data form a FULL RECTANGULAR GRID on (Mach × DeploymentPercentage).
 * - Interpolation: bilinear (stable/monotone); no overshoot like bicubic.
 * - Extrapolation: policy chosen per instance (CONSTANT clamp is recommended).
 *
 * Storage layout:
 *   xs[i]  = unique sorted Mach values, length nx
 *   ys[j]  = unique sorted Deployment values (0..1 or %/100), length ny
 *   z[j][i]= ΔDrag_N at (xs[i], ys[j])
 *
 * Evaluation:
 *   value(x, y) -> ΔDrag_N [N]
 */
public final class GenericFunction2D {

    private final double[] xs;       // ascending Mach grid
    private final double[] ys;       // ascending Deployment grid
    private final double[][] z;      // z[j][i] with shape [ny][nx]
    private final ExtrapolationType xtrap;

    private GenericFunction2D(double[] xs, double[] ys, double[][] z, ExtrapolationType xtrap) {
        this.xs = xs;
        this.ys = ys;
        this.z = z;
        this.xtrap = xtrap;
        sanityCheck();
    }

    // ---------- Public factory: load from CSV ----------

    /**
     * Load a 2D drag surface from CSV with headers:
     *   Mach, DeploymentPercentage, DeltaDrag_N
     * Header names are case-insensitive; commas only (no quoted commas supported).
     */
    public static GenericFunction2D fromCsv(Path csvPath, ExtrapolationType xtrap) throws IOException {
        try (BufferedReader br = Files.newBufferedReader(csvPath, StandardCharsets.UTF_8)) {
            String headerLine = readNonEmptyLine(br);
            if (headerLine == null) {
                throw new IllegalArgumentException("CSV is empty: " + csvPath);
            }

            String[] headers = splitCsv(headerLine);
            Map<String,Integer> hmap = headerIndexMap(headers);

            int iMach   = pick(hmap, List.of("mach"));
            int iDeploy = pick(hmap, List.of("deploymentpercentage"));
            int iValue  = pick(hmap, List.of("deltadrag_n"));

            // Collect rows
            ArrayList<Row> rows = new ArrayList<>();
            for (String line; (line = br.readLine()) != null; ) {
                line = line.trim();
                if (line.isEmpty()) continue;
                String[] toks = splitCsv(line);
                if (toks.length < headers.length) {
                    // allow trailing empties if any
                    toks = Arrays.copyOf(toks, headers.length);
                }
                double mach   = parse(toks[iMach], "Mach");
                double deploy = parse(toks[iDeploy], "DeploymentPercentage");
                double valN   = parse(toks[iValue], "DeltaDrag_N");
                rows.add(new Row(mach, deploy, valN));
            }

            if (rows.isEmpty()) {
                throw new IllegalArgumentException("CSV has headers but no data: " + csvPath);
            }

            // Build unique sorted axes
            double[] xs = uniqueSorted(rows.stream().mapToDouble(r -> r.mach).toArray());
            double[] ys = uniqueSorted(rows.stream().mapToDouble(r -> r.depl).toArray());

            int nx = xs.length, ny = ys.length;
            double[][] z = new double[ny][nx];
            boolean[][] seen = new boolean[ny][nx];

            // Fill grid
            for (Row r : rows) {
                int i = Arrays.binarySearch(xs, r.mach);
                int j = Arrays.binarySearch(ys, r.depl);
                if (i < 0 || j < 0) {
                    throw new IllegalStateException("Internal axis search failure.");
                }
                if (seen[j][i]) {
                    // duplicate point: allow last-one-wins but warn
                    // (You can switch to error if you prefer strictness)
                }
                z[j][i] = r.valN;
                seen[j][i] = true;
            }

            // Validate full rectangular grid
            for (int j = 0; j < ny; j++) {
                for (int i = 0; i < nx; i++) {
                    if (!seen[j][i] || !Double.isFinite(z[j][i])) {
                        throw new IllegalArgumentException(
                            "CSV is not a full grid. Missing or non-finite at Mach=" + xs[i]
                            + ", Deployment=" + ys[j]);
                    }
                }
            }

            return new GenericFunction2D(xs, ys, z, xtrap);
        }
    }

    // ---------- Evaluation ----------

    /** Bilinear interpolation with chosen extrapolation policy. Returns ΔDrag [N]. */
    public double value(double mach, double deploy) {
        int nx = xs.length, ny = ys.length;

        // Handle extrapolation policies up-front
        double x = mach, y = deploy;
        switch (xtrap) {
            case ZERO:
                if (x < xs[0] || x > xs[nx - 1] || y < ys[0] || y > ys[ny - 1]) return 0.0;
                break;
            case CONSTANT:
                x = clamp(x, xs[0], xs[nx - 1]);
                y = clamp(y, ys[0], ys[ny - 1]);
                break;
            case NATURAL:
            default:
                // fall through; allow linear behavior on outermost segment
                break;
        }

        // Indices i,i+1 and j,j+1 such that xs[i] <= x <= xs[i+1], same for y
        int i = bracket(xs, x);
        int j = bracket(ys, y);

        // If at upper boundary exact, shift left/down to keep i+1/j+1 valid
        if (i == nx - 1) i = nx - 2;
        if (j == ny - 1) j = ny - 2;

        double x0 = xs[i], x1 = xs[i + 1];
        double y0 = ys[j], y1 = ys[j + 1];

        double tx = (x1 == x0) ? 0.0 : (x - x0) / (x1 - x0);
        double ty = (y1 == y0) ? 0.0 : (y - y0) / (y1 - y0);

        double z00 = z[j][i];
        double z10 = z[j][i + 1];
        double z01 = z[j + 1][i];
        double z11 = z[j + 1][i + 1];

        // Bilinear blend (stable/monotone)
        double z0 = lerp(z00, z10, tx);
        double z1 = lerp(z01, z11, tx);
        return lerp(z0, z1, ty);
    }

    // ---------- Accessors (optional) ----------
    public double[] xAxis() { return Arrays.copyOf(xs, xs.length); }
    public double[] yAxis() { return Arrays.copyOf(ys, ys.length); }
    public double[][] grid() {
        double[][] out = new double[ys.length][xs.length];
        for (int j = 0; j < ys.length; j++) out[j] = Arrays.copyOf(z[j], xs.length);
        return out;
    }

    // ---------- Helpers ----------

    private void sanityCheck() {
        if (xs.length < 2 || ys.length < 2) {
            throw new IllegalArgumentException("Grid must be at least 2×2 for interpolation.");
        }
        for (int i = 1; i < xs.length; i++) {
            if (!(xs[i] > xs[i - 1])) {
                throw new IllegalArgumentException("Mach axis must be strictly increasing.");
            }
        }
        for (int j = 1; j < ys.length; j++) {
            if (!(ys[j] > ys[j - 1])) {
                throw new IllegalArgumentException("Deployment axis must be strictly increasing.");
            }
        }
        for (int j = 0; j < ys.length; j++) {
            for (int i = 0; i < xs.length; i++) {
                if (!Double.isFinite(z[j][i])) {
                    throw new IllegalArgumentException("Non-finite grid value at [" + j + "," + i + "]");
                }
            }
        }
    }

    private static String readNonEmptyLine(BufferedReader br) throws IOException {
        String s;
        while ((s = br.readLine()) != null) {
            s = s.trim();
            if (!s.isEmpty()) return s;
        }
        return null;
    }

    private static String[] splitCsv(String line) {
        // Simple CSV splitter: comma-separated, no quoted commas supported.
        return line.split(",", -1);
    }

    private static Map<String,Integer> headerIndexMap(String[] headers) {
        Map<String,Integer> map = new HashMap<>();
        for (int i = 0; i < headers.length; i++) {
            map.put(clean(headers[i]), i);
        }
        return map;
    }

    private static int pick(Map<String,Integer> hdr, List<String> names) {
        for (String n : names) {
            Integer idx = hdr.get(clean(n));
            if (idx != null) return idx;
        }
        throw new IllegalArgumentException("CSV missing required column. Need: " + names);
    }

    private static String clean(String s) {
        return s.toLowerCase(Locale.ROOT).trim()
                .replace("%","")
                .replace("_","")
                .replace("-","")
                .replace(" ","");
    }

    private static double parse(String s, String name) {
        try {
            return Double.parseDouble(s.trim());
        } catch (Exception e) {
            throw new IllegalArgumentException("Failed to parse column '" + name + "' value: '" + s + "'");
        }
    }

    private static double[] uniqueSorted(double[] vals) {
        Arrays.sort(vals);
        double[] tmp = new double[vals.length];
        int k = 0;
        double prev = Double.NaN;
        for (double v : vals) {
            if (k == 0 || v != prev) {
                tmp[k++] = v;
                prev = v;
            }
        }
        return Arrays.copyOf(tmp, k);
    }

    private static int bracket(double[] axis, double v) {
        // returns index i such that axis[i] <= v <= axis[i+1], clamped to [0, n-2]
        int n = axis.length;
        if (v <= axis[0]) return 0;
        if (v >= axis[n - 1]) return n - 2;
        int i = Arrays.binarySearch(axis, v);
        if (i >= 0) {
            // exact hit -> use this cell's left edge when possible
            return (i == n - 1) ? n - 2 : Math.max(0, i);
        } else {
            int ip = -i - 1; // insertion point: axis[ip-1] < v < axis[ip]
            return Math.max(0, Math.min(ip - 1, n - 2));
        }
    }

    private static double clamp(double v, double lo, double hi) {
        return (v < lo) ? lo : (v > hi ? hi : v);
    }

    private static double lerp(double a, double b, double t) {
        return a + (b - a) * t;
    }

    // Internal row holder
    private static final class Row {
        final double mach, depl, valN;
        Row(double mach, double depl, double valN) {
            this.mach = mach; this.depl = depl; this.valN = valN;
        }
    }
}
