package com.airbrakesplugin.util;

import java.io.BufferedReader;
import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.*;

/**
 * GenericFunction2D — flexible 2D surface for Airbrake Drag (absolute force in Newtons).
 *
 * Columns (detected generically; case/spacing/punct-insensitive):
 *  - Mach: mach, machnumber, m
 *  - Deployment: deployment, deploymentpercentage, deploymentfraction, deploy, extension, airbrakeext
 *  - Drag [N]: drag, drag_n, dragforce, force, deltadrag (all treated as absolute drag)
 *    NOTE: columns containing "cd" are ignored (we do NOT read coefficients).
 *
 * Accepts scattered data; builds a rectangular grid internally (IDW) for bilinear interpolation.
 */
public final class GenericFunction2D {

    private final double[] xs;       // Mach grid (ascending)
    private final double[] ys;       // Deployment grid (ascending, 0..1)
    private final double[][] z;      // z[j][i] = Drag [N] at (xs[i], ys[j])
    private final ExtrapolationType xtrap;

    private GenericFunction2D(double[] xs, double[] ys, double[][] z, ExtrapolationType xtrap) {
        this.xs = xs;
        this.ys = ys;
        this.z = z;
        this.xtrap = xtrap;
        sanityCheck();
    }

    /** Load Airbrake Drag surface from CSV (comma/semicolon/tab; quotes supported). */
    public static GenericFunction2D fromCsv(Path csvPath, ExtrapolationType xtrap) throws IOException {
        try (BufferedReader br = Files.newBufferedReader(csvPath, StandardCharsets.UTF_8)) {
            String headerLine = readNonEmptyLine(br);
            if (headerLine == null) throw new IllegalArgumentException("CSV is empty: " + csvPath);

            char delim = detectDelimiter(headerLine);
            String[] headers = splitFlexible(headerLine, delim);
            if (headers.length < 3) {
                throw new IllegalArgumentException("CSV header has fewer than 3 columns: " + Arrays.toString(headers));
            }

            Map<String,Integer> hmap = headerIndexMap(headers);
            int iMach   = pick(hmap, List.of("mach","machnumber","m"));
            int iDeploy = pick(hmap, List.of("deployment","deploymentpercentage","deploymentfraction","deploy","extension","airbrakeext"));
            int iDrag   = pickDragColumn(hmap); // robust, ignores Cd columns

            ArrayList<Row> raw = new ArrayList<>();
            for (String line; (line = br.readLine()) != null; ) {
                line = line.trim(); if (line.isEmpty()) continue;
                String[] toks = splitFlexible(line, delim);
                if (toks.length < headers.length) toks = Arrays.copyOf(toks, headers.length);

                String sM = safeGet(toks, iMach);
                String sD = safeGet(toks, iDeploy);
                String sF = safeGet(toks, iDrag);
                if (isBlank(sM) || isBlank(sD) || isBlank(sF)) continue;

                double M = parseNumber(sM, "Mach");
                double dep = parseNumber(sD, "Deployment");
                double F  = parseNumber(sF, headers[iDrag]); // absolute drag [N]
                if (!Double.isFinite(M) || !Double.isFinite(dep) || !Double.isFinite(F)) continue;

                raw.add(new Row(M, dep, F));
            }
            
            if (raw.isEmpty()) throw new IllegalArgumentException("CSV has headers but no valid numeric rows: " + csvPath);

            normalizeDeploymentInPlace(raw);        // 0–100 → 0–1 if needed
            double[] xs = uniqueSorted(raw.stream().mapToDouble(r -> r.mach).toArray());
            double[] ys = uniqueSorted(raw.stream().mapToDouble(r -> r.depl).toArray());
            if (xs.length < 2 || ys.length < 2)
                throw new IllegalArgumentException("Need ≥2 distinct Mach and ≥2 distinct Deployment values. Got nx=" + xs.length + " ny=" + ys.length);

            double[][] z = new double[ys.length][xs.length];
            for (int j = 0; j < ys.length; j++) {
                for (int i = 0; i < xs.length; i++) {
                    Double exact = findExact(raw, xs[i], ys[j]);
                    z[j][i] = (exact != null) ? exact : idw(raw, xs[i], ys[j], 8, 2.0, 1e-12);
                }
            }
            return new GenericFunction2D(xs, ys, z, xtrap);
        }
    }

    /** Returns Airbrake Drag [N] at (mach, deploy). */
    public double dragN(double mach, double deploy) {
        int nx = xs.length, ny = ys.length;
        double x = mach, y = deploy;

        switch (xtrap) {
            case ZERO:
                if (x < xs[0] || x > xs[nx-1] || y < ys[0] || y > ys[ny-1]) return 0.0;
                break;
            case CONSTANT:
                x = clamp(x, xs[0], xs[nx-1]);
                y = clamp(y, ys[0], ys[ny-1]);
                break;
            case NATURAL:
            default: break;
        }

        int i = bracket(xs, x), j = bracket(ys, y);
        if (i == nx - 1) i = nx - 2;
        if (j == ny - 1) j = ny - 2;

        double x0 = xs[i], x1 = xs[i+1];
        double y0 = ys[j], y1 = ys[j+1];
        double tx = (x1 == x0) ? 0.0 : (x - x0)/(x1 - x0);
        double ty = (y1 == y0) ? 0.0 : (y - y0)/(y1 - y0);

        double z00 = z[j][i], z10 = z[j][i+1];
        double z01 = z[j+1][i], z11 = z[j+1][i+1];
        double z0 = lerp(z00, z10, tx);
        double z1 = lerp(z01, z11, tx);
        return lerp(z0, z1, ty);
    }

    // Optional accessors
    public double[] xAxis() { return Arrays.copyOf(xs, xs.length); }
    public double[] yAxis() { return Arrays.copyOf(ys, ys.length); }
    public double[][] grid() {
        double[][] out = new double[ys.length][xs.length];
        for (int j = 0; j < ys.length; j++) out[j] = Arrays.copyOf(z[j], xs.length);
        return out;
    }

    // internals (same as earlier flexible loader) 
    private void sanityCheck() {
        if (xs.length < 2 || ys.length < 2) throw new IllegalArgumentException("Grid must be at least 2×2.");
        for (int i = 1; i < xs.length; i++) if (!(xs[i] > xs[i-1])) throw new IllegalArgumentException("Mach axis must increase.");
        for (int j = 1; j < ys.length; j++) if (!(ys[j] > ys[j-1])) throw new IllegalArgumentException("Deployment axis must increase.");
        for (int j = 0; j < ys.length; j++) for (int i = 0; i < xs.length; i++)
            if (!Double.isFinite(z[j][i])) throw new IllegalArgumentException("Non-finite at ["+j+","+i+"]");
    }

    private static String readNonEmptyLine(BufferedReader br) throws IOException {
        String s; while ((s = br.readLine()) != null) { s = s.trim(); if (!s.isEmpty()) return s; } return null;
    }
    
    private static boolean isBlank(String s){ return s==null || s.trim().isEmpty(); }
    private static String safeGet(String[] a,int i){ return (i>=0 && i<a.length)?a[i]:null; }
    private static char detectDelimiter(String h){ int c=c(h,','), s=c(h,';'), t=c(h,'\t'); return (c>=s && c>=t)?',':(s>=c && s>=t)?';':'\t'; }
    private static int c(String s,char ch){ int k=0; for(int i=0;i<s.length();i++) if(s.charAt(i)==ch) k++; return k; }

    private static String[] splitFlexible(String line,char delim){
        ArrayList<String> out=new ArrayList<>(); StringBuilder sb=new StringBuilder(64); boolean inQ=false;
        for (int i=0;i<line.length();i++){ char ch=line.charAt(i);
            if (ch=='"'){ inQ=!inQ; } else if (ch==delim && !inQ){ out.add(sb.toString()); sb.setLength(0); } else { sb.append(ch); } }
        out.add(sb.toString()); return out.toArray(new String[0]);
    }

    private static Map<String,Integer> headerIndexMap(String[] headers){
        Map<String,Integer> m=new HashMap<>(); for(int i=0;i<headers.length;i++) m.put(clean(headers[i]), i); return m;
    }
    
    private static int pick(Map<String,Integer> hdr, List<String> names){
        for(String n:names){ Integer idx=hdr.get(clean(n)); if(idx!=null) return idx; }
        for (Map.Entry<String,Integer> e: hdr.entrySet()) for(String n:names) if (e.getKey().contains(clean(n))) return e.getValue();
        throw new IllegalArgumentException("CSV missing column. Need one of: "+names+" | headers="+hdr.keySet());
    }
    
    private static int pickDragColumn(Map<String,Integer> hdr){
        String best=null;
        for (String k: hdr.keySet()){
            String ck=k;
            if (ck.contains("cd")) continue; // ignore coefficient columns
            if (ck.contains("deltadrag") || ck.contains("dragforce") || ck.equals("drag") || ck.equals("force") || ck.equals("dragn")) { best=k; break; }
            if (ck.contains("drag")) best=k;
        }
        if (best==null) throw new IllegalArgumentException("CSV missing drag/force column (not Cd). Headers="+hdr.keySet());
        return hdr.get(best);
    }
    
    private static String clean(String s){ return s.toLowerCase(Locale.ROOT).trim().replace("%","").replace("_","").replace("-","").replace(" ",""); }
    private static double parseNumber(String s,String name){ try{ return Double.parseDouble(s.trim()); } catch(Exception e){ throw new IllegalArgumentException("Failed to parse '"+name+"': '"+s+"'"); } }

    private static double[] uniqueSorted(double[] v){
        Arrays.sort(v); double[] t=new double[v.length]; int k=0; boolean first=true; double prev=0;
        for(double x:v){ if(first || x!=prev){ t[k++]=x; prev=x; first=false; } }
        return Arrays.copyOf(t,k);
    }
    private static int bracket(double[] a,double v) { 
        int n=a.length; if (v<=a[0]) return 0; 
        
        if (v>=a[n-1]) return n-2;
        int i=Arrays.binarySearch(a,v); 
        
        if (i>=0) return (i==n-1)? n-2 : Math.max(0,i);
        int ip=-i-1; return Math.max(0, Math.min(ip-1, n-2)); 
    }
    
    private static double clamp(double v,double lo,double hi){ return (v<lo)?lo: (v>hi?hi:v); }
    private static double lerp(double a,double b,double t){ return a + (b-a)*t; }

    private static Double findExact(List<Row> rows,double xm,double yd){ for(Row r:rows) if(r.mach==xm && r.depl==yd) return r.valN; return null; }
    private static double idw(List<Row> rows,double xm,double yd,int K,double p,double eps){
        ArrayList<Node> ns=new ArrayList<>(rows.size());
        for(Row r:rows){ 
            double dx=xm-r.mach, dy=yd-r.depl, d2=dx*dx+dy*dy; 
            if (d2<=eps*eps) return r.valN; 
            ns.add(new Node(Math.sqrt(d2), r.valN)); 
        }
        
        ns.sort(Comparator.comparingDouble(n->n.d)); 
        int use=Math.min(K, ns.size());
        
        double wsum=0, vsum=0; 
        for(int i=0;i<use;i++){ 
            Node n=ns.get(i); 
            double w=1.0/Math.pow(n.d+eps, p); 
            wsum+=w; vsum+=w*n.v; 
        }

        if (wsum==0){ double s=0; 
            for(int i=0;i<use;i++) s+=ns.get(i).v; 
            return s/Math.max(1,use); }
        return vsum/wsum;
    }
    private static void normalizeDeploymentInPlace(List<Row> rows){
        double[] a=new double[rows.size()];
        for(int i=0;i<rows.size();i++) a[i]=rows.get(i).depl;
        Arrays.sort(a); 
        double med=a[a.length/2]; 
        if (med>1.01) for(Row r:rows) r.depl/=100.0;
    }

    private static final class Row { final double mach; double depl; final double valN; Row(double m,double d,double v){mach=m;depl=d;valN=v;} }
    private static final class Node { final double d,v; Node(double d,double v){this.d=d;this.v=v;} }
}
