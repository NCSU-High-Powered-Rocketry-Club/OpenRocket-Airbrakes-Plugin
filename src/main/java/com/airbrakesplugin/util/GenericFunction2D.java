package com.airbrakesplugin.util;

import org.apache.commons.csv.CSVFormat;
import org.apache.commons.csv.CSVParser;
import org.apache.commons.csv.CSVRecord;
import org.apache.commons.math3.analysis.BivariateFunction;
import org.apache.commons.math3.analysis.interpolation.BicubicInterpolator;

import java.io.IOException;
import java.io.Reader;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;
import java.util.Set;
import java.util.TreeSet;

/**
 * Loads (mach, deployment) → value from a CSV and builds a bicubic interpolator.
 * Handles extrapolation per {@link ExtrapolationType}.
 */
public class GenericFunction2D implements Function2D {
    private final Path csvPath;
    private final String valueColumn;
    private final ExtrapolationType extrapolation;
    private volatile BivariateFunction interp;
    private double minX, maxX, minY, maxY;

    public GenericFunction2D(Path csvPath, String valueColumn, ExtrapolationType extrapolation) {
        this.csvPath       = csvPath;
        this.valueColumn   = valueColumn;
        this.extrapolation = extrapolation;
    }

    @Override
    public double value(double x, double y) {
        ensureInterpolatorBuilt();

        boolean outX = x < minX || x > maxX;
        boolean outY = y < minY || y > maxY;
        if (outX || outY) {
            switch (extrapolation) {
                case ZERO:
                    return 0.0;
                case CONSTANT:
                    x = clamp(x, minX, maxX);
                    y = clamp(y, minY, maxY);
                    break;
                case NATURAL:
                    // trust the interpolator to extrapolate
                    break;
            }
        }
        return interp.value(x, y);
    }

    private synchronized void ensureInterpolatorBuilt() {
        if (interp != null) return;

        List<Double> xList = new ArrayList<>();
        List<Double> yList = new ArrayList<>();
        List<Record> records = new ArrayList<>();

        try (Reader reader = Files.newBufferedReader(csvPath);
             CSVParser parser = CSVFormat.DEFAULT
                                 .builder()
                                 .setHeader()
                                 .setSkipHeaderRecord(true)
                                 .build()
                                 .parse(reader)) {
            for (CSVRecord rec : parser) {
                double mach   = Double.parseDouble(rec.get("Mach"));
                double deploy = Double.parseDouble(rec.get("DeploymentPercentage"));
                double val    = Double.parseDouble(rec.get(valueColumn));
                xList.add(mach);
                yList.add(deploy);
                records.add(new Record(mach, deploy, val));
            }
        } catch (IOException e) {
            throw new RuntimeException("Failed to read CSV at " + csvPath, e);
        }

        // unique sorted axes
        Set<Double> xSet = new TreeSet<>(xList);
        Set<Double> ySet = new TreeSet<>(yList);
        double[] xs = xSet.stream().mapToDouble(Double::doubleValue).toArray();
        double[] ys = ySet.stream().mapToDouble(Double::doubleValue).toArray();

        minX = xs[0];           maxX = xs[xs.length-1];
        minY = ys[0];           maxY = ys[ys.length-1];

        // fill grid f[i][j] = f(xs[i], ys[j])
        double[][] f = new double[xs.length][ys.length];
        for (Record r : records) {
            int i = idx(xs, r.x);
            int j = idx(ys, r.y);
            f[i][j] = r.value;
        }

        // build interpolator
        interp = new BicubicInterpolator().interpolate(xs, ys, f);
    }

    private int idx(double[] arr, double v) {
        for (int i = 0; i < arr.length; i++) {
            if (arr[i] == v) return i;
        }
        throw new IllegalArgumentException("Value "+v+" not in axis");
    }

    private double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }

    private static class Record {
        final double x, y, value;
        Record(double x, double y, double value) {
            this.x     = x;
            this.y     = y;
            this.value = value;
        }
    }
}
