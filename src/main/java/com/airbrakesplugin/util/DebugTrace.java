package com.airbrakesplugin.util;

import javax.swing.*;
import javax.swing.text.DefaultCaret;
import java.awt.*;
import java.io.*;
import java.nio.charset.StandardCharsets;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.LinkedList;
import java.util.Locale;
import java.util.UUID;

public final class DebugTrace {

    public static final class PredictorSample {
        public final double t, alt, vWorldZ, aWorldZ, estApStrict, estApBest, uncert;
        public final int packets;
        public final String note;
        public PredictorSample(double t, double alt, double vWorldZ, double aWorldZ,
                               double estApStrict, double estApBest, double uncert,
                               int packets, String note) {
            this.t = t; this.alt = alt; this.vWorldZ = vWorldZ; this.aWorldZ = aWorldZ;
            this.estApStrict = estApStrict; this.estApBest = estApBest; this.uncert = uncert;
            this.packets = packets; this.note = (note == null ? "" : note);
        }
    }

    public static final class ControllerSample {
        public final double t, alt, target, deadband, apogeeEst, error, deployCmd;
        public final String reason;
        public ControllerSample(double t, double alt, double target, double deadband,
                                double apogeeEst, double error, double deployCmd, String reason) {
            this.t = t; this.alt = alt; this.target = target; this.deadband = deadband;
            this.apogeeEst = apogeeEst; this.error = error; this.deployCmd = deployCmd;
            this.reason = (reason == null ? "" : reason);
        }
    }

    private final boolean enabled;
    private final boolean writeCsv;
    private final boolean showConsole;

    private final LinkedList<String> ring = new LinkedList<>();
    private final int ringMax = 2000;

    private final File predCsv;
    private final File ctrlCsv;
    private volatile boolean wrotePredHeader = false;
    private volatile boolean wroteCtrlHeader = false;

    private JTextArea consoleArea;

    private static String nowStr() {
        return new SimpleDateFormat("yyyyMMdd_HHmmss", Locale.US).format(new Date());
    }

    public DebugTrace(boolean enabled, boolean writeCsv, boolean showConsole, String csvDirHint, String simTag) {
        this.enabled = enabled;
        this.writeCsv = enabled && writeCsv;
        this.showConsole = enabled && showConsole;

        final String base = (simTag == null || simTag.isBlank()) ? UUID.randomUUID().toString() : simTag;
        final File dir = (csvDirHint != null && !csvDirHint.isBlank()) ? new File(csvDirHint) : new File(System.getProperty("java.io.tmpdir"));
        if (writeCsv) dir.mkdirs();
        this.predCsv = writeCsv ? new File(dir, "pred_" + base + "_" + nowStr() + ".csv") : null;
        this.ctrlCsv = writeCsv ? new File(dir, "ctrl_" + base + "_" + nowStr() + ".csv") : null;

        if (this.showConsole) SwingUtilities.invokeLater(this::ensureConsole);
    }

    private void ensureConsole() {
        if (consoleArea != null) return;
        final JFrame f = new JFrame("Airbrakes Debug Console");
        f.setDefaultCloseOperation(WindowConstants.DISPOSE_ON_CLOSE);
        consoleArea = new JTextArea(30, 110);
        consoleArea.setEditable(false);
        final DefaultCaret caret = (DefaultCaret) consoleArea.getCaret();
        caret.setUpdatePolicy(DefaultCaret.ALWAYS_UPDATE);
        f.add(new JScrollPane(consoleArea), BorderLayout.CENTER);
        f.pack();
        f.setVisible(true);
    }

    private void append(String line) {
        if (!enabled) return;
        synchronized (ring) {
            ring.add(line);
            while (ring.size() > ringMax) ring.removeFirst();
        }
        if (consoleArea != null) {
            SwingUtilities.invokeLater(() -> consoleArea.append(line + "\n"));
        }
    }

    private static void writeLine(File f, String line, boolean[] headerWritten, String header) {
        try (var out = new BufferedWriter(new OutputStreamWriter(new FileOutputStream(f, true), StandardCharsets.UTF_8))) {
            if (!headerWritten[0]) {
                out.write(header); out.write("\n");
                headerWritten[0] = true;
            }
            out.write(line); out.write("\n");
        } catch (IOException ignored) {}
    }

    public void addPredictor(PredictorSample s) {
        if (!enabled) return;
        final String line = String.format(Locale.US,
                "PRED,t=%.4f,alt=%.3f,vz=%.3f,az=%.3f,apo_strict=%.3f,apo_best=%.3f,unc=%.4f,pack=%d,%s",
                s.t, s.alt, s.vWorldZ, s.aWorldZ, s.estApStrict, s.estApBest, s.uncert, s.packets, s.note);
        append(line);

        if (writeCsv && predCsv != null) {
            final boolean[] hdr = new boolean[]{wrotePredHeader};
            writeLine(predCsv,
                    String.format(Locale.US,
                            "%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%d,%s",
                            s.t, s.alt, s.vWorldZ, s.aWorldZ, s.estApStrict, s.estApBest, s.uncert, s.packets, s.note.replace(',', ';')),
                    hdr,
                    "t,alt,vz,az,apo_strict,apo_best,uncert,packets,note");
            wrotePredHeader = hdr[0];
        }
    }

    public void addController(ControllerSample s) {
        if (!enabled) return;
        final String line = String.format(Locale.US,
                "CTRL,t=%.4f,alt=%.3f,target=%.3f,db=%.3f,apo=%.3f,err=%.3f,deploy=%.4f,%s",
                s.t, s.alt, s.target, s.deadband, s.apogeeEst, s.error, s.deployCmd, s.reason);
        append(line);

        if (writeCsv && ctrlCsv != null) {
            final boolean[] hdr = new boolean[]{wroteCtrlHeader};
            writeLine(ctrlCsv,
                    String.format(Locale.US,
                            "%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%s",
                            s.t, s.alt, s.target, s.deadband, s.apogeeEst, s.error, s.deployCmd, s.reason.replace(',', ';')),
                    hdr,
                    "t,alt,target,deadband,apo_est,error,deploy_cmd,reason");
            wroteCtrlHeader = hdr[0];
        }
    }

    public void note(String msg) { append("NOTE," + msg); }
}
