package com.airbrakesplugin.util;

import org.apache.commons.csv.*;
import org.apache.commons.math3.analysis.BivariateFunction;
import org.apache.commons.math3.analysis.interpolation.BicubicInterpolator;

import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.*;
import java.util.*;

public final class GenericFunction2D {

    /* header aliases */
    private static final List<String> MACH_HEADERS   = List.of("mach", "m");
    private static final List<String> DEPLOY_HEADERS = List.of(
            "deploymentpercentage", "deploymentpct", "deploymentlevel",
            "deployment", "deploypct", "deployment_percentage");
    private static final List<String> CD_HEADERS     = List.of("cdincrement", "cd", "dcd");
    private static final List<String> CM_HEADERS     = List.of("cmincrement", "cm", "dcm");

    /* ------------------------------------------------------------------ */
    private final Path csvFile;
    private volatile BivariateFunction cdFunc;
    private volatile BivariateFunction cmFunc;
    private double[] machAxis, deployAxis;

    public GenericFunction2D(Path csvFile) { this.csvFile = Objects.requireNonNull(csvFile); }

    /* public look-ups --------------------------------------------------- */
    public double valueCd(double mach, double deploy) {
        ensureBuilt();
        return cdFunc.value(clamp(mach, machAxis), clamp(deploy, deployAxis));
    }
    public double valueCm(double mach, double deploy) {
        ensureBuilt();
        return cmFunc.value(clamp(mach, machAxis), clamp(deploy, deployAxis));
    }

    /* ------------------------------------------------------------------ */
    private synchronized void ensureBuilt() {
        if (cdFunc != null) return;
        if (!Files.isReadable(csvFile))
            throw new IllegalStateException("CFD CSV unreadable: " + csvFile);

        CSVFormat fmt = CSVFormat.Builder.create()
                .setHeader().setSkipHeaderRecord(true).setTrim(true).build();

        /* Map<Mach, Map<Deploy, [dCd,dCm]>> */
        Map<Double, Map<Double, double[]>> tbl = new TreeMap<>();

        try (CSVParser p = CSVParser.parse(csvFile, StandardCharsets.UTF_8, fmt)) {
            Map<String,Integer> hdr = normaliseHeaders(p.getHeaderMap());

            String kMach   = pick(hdr, MACH_HEADERS,   "Mach");
            String kDep    = pick(hdr, DEPLOY_HEADERS, "deployment-percentage");
            String kCd     = pick(hdr, CD_HEADERS,     "ΔCd");
            String kCm     = pick(hdr, CM_HEADERS,     "ΔCm");

            for (CSVRecord r : p) {
                double m  = Double.parseDouble(r.get(hdr.get(kMach)));
                double d  = Double.parseDouble(r.get(hdr.get(kDep)));
                double dCd= Double.parseDouble(r.get(hdr.get(kCd)));
                double dCm= Double.parseDouble(r.get(hdr.get(kCm)));

                tbl.computeIfAbsent(m, k -> new TreeMap<>())
                   .put(d, new double[]{dCd, dCm});
            }
        } catch (IOException ioe) {
            throw new IllegalStateException("Error reading CFD CSV: " + csvFile, ioe);
        }

        /* ---------- axes (ensure 0-% row exists) ------------------------ */
        machAxis   = tbl.keySet().stream().mapToDouble(Double::doubleValue).toArray();

        Set<Double> deploySet = new TreeSet<>();
        tbl.values().forEach(m -> deploySet.addAll(m.keySet()));
        deploySet.add(0.0);                                  // force 0-row
        deployAxis = deploySet.stream().mapToDouble(Double::doubleValue).toArray();

        int nx = machAxis.length, ny = deployAxis.length;
        double[][] zCd = new double[nx][ny];
        double[][] zCm = new double[nx][ny];

        for (int i = 0; i < nx; i++) {
            Map<Double,double[]> row = tbl.get(machAxis[i]);
            for (int j = 0; j < ny; j++) {
                double dep = deployAxis[j];
                double[] v = row.get(dep);

                if (v == null) {
                    /* special-case: missing 0-% gets [0,0]; others → nearest */
                    v = (dep == 0.0)
                            ? new double[]{0.0, 0.0}
                            : nearest(tbl, machAxis[i], dep);
                }
                zCd[i][j] = v[0];
                zCm[i][j] = v[1];
            }
        }

        BicubicInterpolator interp = new BicubicInterpolator();
        cdFunc = interp.interpolate(machAxis, deployAxis, zCd);
        cmFunc = interp.interpolate(machAxis, deployAxis, zCm);
    }

    /* nearest-neighbour fallback */
    private static double[] nearest(Map<Double, Map<Double, double[]>> tbl,
                                    double mach, double dep) {
        double best = Double.MAX_VALUE;
        double[] val = null;
        for (var e1 : tbl.entrySet()) {
            double dm = e1.getKey() - mach;
            for (var e2 : e1.getValue().entrySet()) {
                double dd = e2.getKey() - dep;
                double dist = dm*dm + dd*dd;
                if (dist < best) { best = dist; val = e2.getValue(); }
            }
        }
        return val;
    }

    /* helpers */
    private static Map<String,Integer> normaliseHeaders(Map<String,Integer> raw) {
        Map<String,Integer> n = new HashMap<>();
        raw.forEach((k,v) -> n.put(
                k.toLowerCase(Locale.ROOT)
                 .replace("%","").replace("_","")
                 .replace("-","").replace(" ",""),
                v));
        return n;
    }
    private static String pick(Map<String,Integer> hdr, List<String> alias, String name) {
        return alias.stream().filter(hdr::containsKey).findFirst()
                .orElseThrow(() -> new IllegalArgumentException(
                        "CSV missing " + name + " column (found " + hdr.keySet() + ')'));
    }
    private static double clamp(double v, double[] axis) {
        return Math.max(axis[0], Math.min(v, axis[axis.length-1]));
    }
}
