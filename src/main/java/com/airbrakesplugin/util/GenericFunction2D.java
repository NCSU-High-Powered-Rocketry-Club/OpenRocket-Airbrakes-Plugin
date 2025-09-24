package com.airbrakesplugin.util;

import java.io.BufferedReader;
import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.*;

/**
 * GenericFunction2D — flexible 2D surface for ΔDrag (force in Newtons).
 *
 * Improvements vs. the strict loader:
 *  - Accepts ANY set of Mach values and ANY set of Deployment levels (no "full grid" required).
 *  - Header names are detected generically (case/space/punctuation-insensitive).
 *  - Supports comma, semicolon, or tab delimiters; quoted fields allowed.
 *  - Deployment auto-normalization: if median(deployment) > 1.0, assumes percent and divides by 100.
 *  - Missing grid cells are prefilled using IDW (inverse distance weighting) from the provided scattered points.
 *
 * Internal storage remains a rectangular grid for fast bilinear interpolation and extrapolation handling.
 *
 * Columns (detected generically; examples):
 *  - Mach: mach, machnumber, m
 *  - Deployment: deployment, deploymentpercentage, deploymentfraction, deploy, extension, airbrakeext
 *  - Drag force [N]: deltadrag, deltadrag_n, drag, dragforce, force   (NOTE: columns named "Cd" are ignored)
 */
public final class GenericFunction2D {

    private final double[] xs;       // ascending Mach grid
    private final double[] ys;       // ascending Deployment grid
    private final double[][] z;      // z[j][i] with shape [ny][nx], ΔDrag_N
    private final ExtrapolationType xtrap;

    private GenericFunction2D(double[] xs, double[] ys, double[][] z, ExtrapolationType xtrap) {
        this.xs = xs;
        this.ys = ys;
        this.z = z;
        this.xtrap = xtrap;
        sanityCheck();
    }

    // ---------- Public factory: load from CSV (flexible) ----------

    public static GenericFunction2D fromCsv(Path csvPath, ExtrapolationType xtrap) throws IOException {
        try (BufferedReader br = Files.newBufferedReader(csvPath, StandardCharsets.UTF_8)) {
            // Read first non-empty line as header
            String headerLine = readNonEmptyLine(br);
            if (headerLine == null) {
                throw new IllegalArgumentException("CSV is empty: " + csvPath);
            }

            // Detect delimiter
            char delim = detectDelimiter(headerLine);

            // Split headers (support quotes)
            String[] headers = splitFlexible(headerLine, delim);
            if (headers.length < 3) {
                throw new IllegalArgumentException("CSV header has fewer than 3 columns: " + Arrays.toString(headers));
            }

            // Map headers (generic detection)
            Map<String,Integer> hmap = headerIndexMap(headers);

            int iMach   = pick(hmap, List.of("mach","machnumber","m"));
            int iDeploy = pick(hmap, List.of("deployment","deploymentpercentage","deploymentfraction","deploy","extension","airbrakeext"));
            int iValue  = pickDragColumn(hmap); // robust drag/force detection, ignoring Cd

            // Collect rows (scattered allowed)
            ArrayList<Row> raw = new ArrayList<>();
            for (String line; (line = br.readLine()) != null; ) {
                line = line.trim();
                if (line.isEmpty()) continue;
                String[] toks = splitFlexible(line, delim);
                if (toks.length < headers.length) {
                    toks = Arrays.copyOf(toks, headers.length);
                }

                String sMach = safeGet(toks, iMach);
                String sDep  = safeGet(toks, iDeploy);
                String sVal  = safeGet(toks, iValue);

                if (isBlank(sMach) || isBlank(sDep) || isBlank(sVal)) continue; // skip incomplete rows

                double mach   = parseNumber(sMach, "Mach");
                double deploy = parseNumber(sDep,  "Deployment");
                double valN   = parseNumber(sVal,  headers[iValue]);

                if (!Double.isFinite(mach) || !Double.isFinite(deploy) || !Double.isFinite(valN)) continue;

                raw.add(new Row(mach, deploy, valN));
            }

            if (raw.isEmpty()) {
                throw new IllegalArgumentException("CSV has headers but no valid numeric rows: " + csvPath);
            }

            // Normalize deployment to fraction [0..1] if it looks like percent
            normalizeDeploymentInPlace(raw);

            // Build unique sorted axes directly from provided values (no grid required in file)
            double[] xs = uniqueSorted(raw.stream().mapToDouble(r -> r.mach).toArray());
            double[] ys = uniqueSorted(raw.stream().mapToDouble(r -> r.depl).toArray());

            if (xs.length < 2 || ys.length < 2) {
                throw new IllegalArgumentException("Need at least 2 distinct Mach values and 2 distinct Deployment levels to interpolate. "
                        + "Got Mach nx=" + xs.length + ", Deploy ny=" + ys.length);
            }

            // Prefill rectangular grid using IDW from scattered points
            double[][] z = new double[ys.length][xs.length];

            // Build kd-ish simple index (just keep the raw list; N is small)
            for (int j = 0; j < ys.length; j++) {
                for (int i = 0; i < xs.length; i++) {
                    double xm = xs[i];
                    double yd = ys[j];

                    // Try exact match first
                    Double exact = findExact(raw, xm, yd);
                    if (exact != null) {
                        z[j][i] = exact;
                        continue;
                    }
                    // Otherwise estimate via IDW (p=2), using up to K nearest points
                    z[j][i] = idw(raw, xm, yd, /*K*/8, /*power*/2.0, /*eps*/1e-12);
                }
            }

            return new GenericFunction2D(xs, ys, z, xtrap);
        }
    }

    // ---------- Evaluation ----------

    /** Bilinear interpolation with chosen extrapolation policy. Returns ΔDrag [N]. */
    public double value(double mach, double deploy) {
        int nx = xs.length, ny = ys.length;

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
                // allow linear behavior on outermost segment
                break;
        }

        int i = bracket(xs, x);
        int j = bracket(ys, y);
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

        double z0 = lerp(z00, z10, tx);
        double z1 = lerp(z01, z11, tx);
        return lerp(z0, z1, ty);
    }

    // ---------- Accessors ----------
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

    private static boolean isBlank(String s) {
        return s == null || s.trim().isEmpty();
    }

    private static String safeGet(String[] arr, int idx) {
        return (idx >= 0 && idx < arr.length) ? arr[idx] : null;
    }

    // Detect delimiter by simple scoring
    private static char detectDelimiter(String headerLine) {
        int c = count(headerLine, ',');
        int s = count(headerLine, ';');
        int t = count(headerLine, '\t');
        if (c >= s && c >= t) return ',';
        if (s >= c && s >= t) return ';';
        return '\t';
    }

    private static int count(String s, char ch) {
        int k = 0;
        for (int i = 0; i < s.length(); i++) if (s.charAt(i) == ch) k++;
        return k;
    }

    // Split supporting quotes and chosen delimiter
    private static String[] splitFlexible(String line, char delim) {
        ArrayList<String> out = new ArrayList<>();
        StringBuilder sb = new StringBuilder(64);
        boolean inQuotes = false;
        for (int i = 0; i < line.length(); i++) {
            char ch = line.charAt(i);
            if (ch == '"') {
                inQuotes = !inQuotes;
            } else if (ch == delim && !inQuotes) {
                out.add(sb.toString());
                sb.setLength(0);
            } else {
                sb.append(ch);
            }
        }
        out.add(sb.toString());
        return out.toArray(new String[0]);
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
        // fallback: find first header that contains the token
        for (Map.Entry<String,Integer> e : hdr.entrySet()) {
            for (String n : names) {
                if (e.getKey().contains(clean(n))) return e.getValue();
            }
        }
        throw new IllegalArgumentException("CSV missing required column. Need one of: " + names + " | headers=" + hdr.keySet());
    }

    // Pick drag/force column robustly, ignoring Cd-like columns
    private static int pickDragColumn(Map<String,Integer> hdr) {
        String bestKey = null;
        for (String k : hdr.keySet()) {
            String ck = k;
            if (ck.contains("cd")) continue; // ignore coefficient columns
            if (ck.contains("deltadrag") || ck.contains("dragforce") || ck.equals("drag") || ck.equals("force")) {
                bestKey = k; break;
            }
        }
        if (bestKey == null) {
            // secondary search: any 'drag' or 'force' not labeled 'cd'
            for (String k : hdr.keySet()) {
                String ck = k;
                if (ck.contains("cd")) continue;
                if (ck.contains("drag") || ck.contains("force")) { bestKey = k; break; }
            }
        }
        if (bestKey == null) {
            throw new IllegalArgumentException("CSV missing drag/force column. Looking for 'DeltaDrag', 'DeltaDrag_N', 'Drag', or 'Force' (not Cd). Headers=" + hdr.keySet());
        }
        return hdr.get(bestKey);
    }

    private static String clean(String s) {
        return s.toLowerCase(Locale.ROOT).trim()
                .replace("%","")
                .replace("_","")
                .replace("-","")
                .replace(" ","");
    }

    private static double parseNumber(String s, String name) {
        try {
            return Double.parseDouble(s.trim());
        } catch (Exception e) {
            throw new IllegalArgumentException("Failed to parse numeric '" + name + "' value: '" + s + "'");
        }
    }

    private static double[] uniqueSorted(double[] vals) {
        Arrays.sort(vals);
        double[] tmp = new double[vals.length];
        int k = 0;
        boolean first = true;
        double prev = 0;
        for (double v : vals) {
            if (first || v != prev) {
                tmp[k++] = v;
                prev = v;
                first = false;
            }
        }
        return Arrays.copyOf(tmp, k);
    }

    private static int bracket(double[] axis, double v) {
        int n = axis.length;
        if (v <= axis[0]) return 0;
        if (v >= axis[n - 1]) return n - 2;
        int i = Arrays.binarySearch(axis, v);
        if (i >= 0) return (i == n - 1) ? n - 2 : Math.max(0, i);
        int ip = -i - 1; // insertion point
        return Math.max(0, Math.min(ip - 1, n - 2));
    }

    private static double clamp(double v, double lo, double hi) {
        return (v < lo) ? lo : (v > hi ? hi : v);
    }

    private static double lerp(double a, double b, double t) {
        return a + (b - a) * t;
    }

    // Exact match lookup
    private static Double findExact(List<Row> rows, double xm, double yd) {
        for (Row r : rows) {
            if (r.mach == xm && r.depl == yd) return r.valN;
        }
        return null;
    }

    // Inverse-distance weighting estimator over scattered data
    private static double idw(List<Row> rows, double xm, double yd, int K, double power, double eps) {
        // collect distances
        ArrayList<Node> nodes = new ArrayList<>(rows.size());
        for (Row r : rows) {
            double dx = xm - r.mach;
            double dy = yd - r.depl;
            double d2 = dx*dx + dy*dy;
            if (d2 <= eps*eps) return r.valN; // very close point
            nodes.add(new Node(Math.sqrt(d2), r.valN));
        }
        // partial sort by distance
        nodes.sort(Comparator.comparingDouble(n -> n.d));
        int use = Math.min(K, nodes.size());
        double wsum = 0.0, vsum = 0.0;
        for (int i = 0; i < use; i++) {
            Node n = nodes.get(i);
            double w = 1.0 / Math.pow(n.d + eps, power);
            wsum += w;
            vsum += w * n.v;
        }
        if (wsum == 0.0) {
            // fallback to simple average of K nearest
            double sum = 0.0;
            for (int i = 0; i < use; i++) sum += nodes.get(i).v;
            return sum / Math.max(1, use);
        }
        return vsum / wsum;
    }

    private static void normalizeDeploymentInPlace(List<Row> rows) {
        // Heuristic: if median deployment > 1.0 by a margin, treat as percent and divide by 100
        double[] a = new double[rows.size()];
        for (int i = 0; i < rows.size(); i++) a[i] = rows.get(i).depl;
        Arrays.sort(a);
        double med = a[a.length / 2];
        if (med > 1.01) {
            for (Row r : rows) r.depl = r.depl / 100.0;
        }
    }

    // Internal holders
    private static final class Row {
        final double mach;
        double depl;
        final double valN;
        Row(double mach, double depl, double valN) {
            this.mach = mach; this.depl = depl; this.valN = valN;
        }
    }
    private static final class Node {
        final double d;
        final double v;
        Node(double d, double v) { this.d = d; this.v = v; }
    }
}
