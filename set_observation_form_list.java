package macro;

import java.io.*;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

import star.common.*;
import star.base.neo.*;
import star.vis.*;

/**
 * add_history_points.java
 *
 * Reads a plain-text file of XYZ probe positions and creates a
 * PointProbe (history point) for each one in the fluid region.
 *
 * Position file format (one probe per line, whitespace-separated):
 *   x  y  z  [optional_name]
 *
 *   Example:
 *     1.0   0.0   0.5   wake_1
 *     2.0   0.5   0.5   wake_2
 *     0.5  -0.5   0.5
 *
 * If no name is given the probe is named "probe_N" (1-indexed).
 * Lines starting with # or // and blank lines are ignored.
 *
 * Config read from case_config.json:
 *   probes.points_file   path to the XYZ list file
 *   probes.region        region name (default: "fluid")
 *   probes.report_Cp     if true, also create Cp scalar reports (default: false)
 *
 * STAR-CCM+ version target: 2602 (2026-02)
 */
public class add_history_points extends StarMacro {

    @Override
    public void execute() {

        Simulation sim = getActiveSimulation();

        /* 1. JSON config -------------------------------------------------- */
        String cfgPath = System.getenv("STARCCM_JSON");
        if (cfgPath == null || cfgPath.trim().isEmpty())
            cfgPath = "F:/STARCCM-UMICH/Nueva Carpeta/project-cylAFC/case_config.JSON";

        JsonObject cfg = Json.parseFile(cfgPath);
        String pointsFile  = cfg.getString("probes.points_file", "");
        String regionName  = cfg.getString("probes.region",      "fluid");

        if (pointsFile.isEmpty())
            throw new RuntimeException("[add_history_points] probes.points_file must be set in JSON.");

        log(sim, "Reading probe positions from: " + pointsFile);
        log(sim, "Target region: " + regionName);

        /* 2. Parse probe file --------------------------------------------- */
        List<double[]>  coords = new ArrayList<double[]>();
        List<String>    names  = new ArrayList<String>();

        try (BufferedReader br = new BufferedReader(new FileReader(pointsFile))) {
            String line;
            int lineNum = 0;
            int probeIdx = 1;
            while ((line = br.readLine()) != null) {
                lineNum++;
                line = line.trim();
                if (line.isEmpty() || line.startsWith("#") || line.startsWith("//")) continue;
                String[] tok = line.split("\\s+");
                if (tok.length < 3) {
                    sim.println("[add_history_points] WARNING: skipping line " + lineNum
                        + " (need at least 3 values): " + line);
                    continue;
                }
                try {
                    double x = Double.parseDouble(tok[0]);
                    double y = Double.parseDouble(tok[1]);
                    double z = Double.parseDouble(tok[2]);
                    String name = (tok.length >= 4) ? tok[3] : ("probe_" + probeIdx);
                    coords.add(new double[]{x, y, z});
                    names.add(name);
                    probeIdx++;
                } catch (NumberFormatException ex) {
                    sim.println("[add_history_points] WARNING: skipping line " + lineNum
                        + " (parse error): " + line);
                }
            }
        } catch (IOException ex) {
            throw new RuntimeException("[add_history_points] Cannot read points file: " + pointsFile, ex);
        }

        if (coords.isEmpty())
            throw new RuntimeException("[add_history_points] No valid probe positions found in: " + pointsFile);

        log(sim, "Found " + coords.size() + " probe positions.");

        /* 3. Get region --------------------------------------------------- */
        Region region = sim.getRegionManager().getRegion(regionName);

        /* 4. Remove any existing probes with the same names --------------- */
        for (String name : names) {
            try {
                PointProbe existing = (PointProbe) sim.getReportManager().getReport(name);
                sim.getReportManager().removeObjects(new ArrayList<Report>() {{ add(existing); }});
                sim.println("[add_history_points] Removed existing probe: " + name);
            } catch (Exception ex) {
                // probe didn't exist — fine
            }
        }

        /* 5. Create probes ------------------------------------------------ */
        Units m = (Units) sim.getUnitsManager().getObject("m");

        for (int i = 0; i < coords.size(); i++) {
            double[] xyz  = coords.get(i);
            String   name = names.get(i);

            try {
                PointProbe probe = sim.getReportManager().createReport(PointProbe.class);
                probe.setPresentationName(name);

                // Set position
                probe.getPoint().setCoordinateSystem(
                    sim.getCoordinateSystemManager().getLabCoordinateSystem());
                probe.getPoint().setUnits0(m);
                probe.getPoint().setUnits1(m);
                probe.getPoint().setUnits2(m);
                probe.getPoint().setValue(new DoubleVector(new double[]{xyz[0], xyz[1], xyz[2]}));

                // Assign to region parts
                probe.getParts().setObjects(region);

                log(sim, String.format(java.util.Locale.US,
                    "Created probe '%s' at (%.4g, %.4g, %.4g)", name, xyz[0], xyz[1], xyz[2]));

            } catch (Exception ex) {
                sim.println("[add_history_points] ERROR creating probe '" + name + "': " + ex.getMessage());
            }
        }

        /* 6. Create a monitor + history plot for each probe --------------- */
        for (int i = 0; i < coords.size(); i++) {
            String name = names.get(i);
            try {
                PointProbe probe = (PointProbe) sim.getReportManager().getReport(name);

                // Create report monitor
                ReportMonitor monitor = (ReportMonitor) sim.getMonitorManager()
                    .createMonitor(ReportMonitor.class);
                monitor.setPresentationName(name + "_monitor");
                monitor.setReport(probe);

            } catch (Exception ex) {
                sim.println("[add_history_points] WARNING: could not create monitor for '"
                    + name + "': " + ex.getMessage());
            }
        }

        log(sim, "Done. Created " + coords.size() + " history point probes.");
    }

    /*----------------------------------------------------------------------*/
    private static void log(Simulation s, String m) {
        s.println("[add_history_points] " + m);
    }

    /*######################################################################*/
    /*  Mini JSON parser                                                   */
    /*######################################################################*/
    static final class Json {
        static JsonObject parseFile(String path) {
            StringBuilder sb = new StringBuilder();
            try (BufferedReader br = new BufferedReader(new FileReader(path))) {
                String line;
                while ((line = br.readLine()) != null) sb.append(line).append('\n');
            } catch (IOException ex) {
                throw new RuntimeException("Cannot read JSON: " + path, ex);
            }
            Object parsed = new Parser(sb.toString()).parseValue();
            if (!(parsed instanceof Map))
                throw new RuntimeException("Top-level JSON must be an object.");
            @SuppressWarnings("unchecked")
            JsonObject obj = new JsonObject((Map<String, Object>) parsed);
            return obj;
        }
    }

    static final class JsonObject {
        private final Map<String, Object> map;
        JsonObject(Map<String, Object> m) { this.map = m; }
        double getDouble(String path, double d) { Object v = get(path); return (v instanceof Number) ? ((Number)v).doubleValue() : d; }
        String getString(String path, String d) { Object v = get(path); return (v instanceof String) ? (String)v : d; }
        private Object get(String path) {
            String[] parts = path.split("\\.");
            Object cur = map;
            for (String p : parts) {
                if (!(cur instanceof Map)) return null;
                cur = ((Map<?,?>)cur).get(p);
                if (cur == null) return null;
            }
            return cur;
        }
    }

    static final class Parser {
        private final String s; private int i = 0;
        Parser(String s) { this.s = s; }
        Object parseValue() {
            skipWs(); if (i >= s.length()) throw err("Unexpected end");
            char c = s.charAt(i);
            if (c=='{') return parseObj(); if (c=='[') return parseArr();
            if (c=='"') return parseStr(); if (c=='t'||c=='f') return parseBool();
            if (c=='n') return parseNull(); return parseNum();
        }
        private Map<String,Object> parseObj() {
            expect('{'); LinkedHashMap<String,Object> out = new LinkedHashMap<String,Object>();
            skipWs(); if (peek('}')) { i++; return out; }
            while (true) { skipWs(); String k=parseStr(); skipWs(); expect(':');
                out.put(k, parseValue()); skipWs(); if (peek('}')) { i++; break; } expect(','); }
            return out;
        }
        private List<Object> parseArr() {
            expect('['); List<Object> out = new ArrayList<Object>();
            skipWs(); if (peek(']')) { i++; return out; }
            while (true) { out.add(parseValue()); skipWs(); if (peek(']')) { i++; break; } expect(','); }
            return out;
        }
        private String parseStr() {
            expect('"'); StringBuilder sb = new StringBuilder();
            while (i < s.length()) {
                char c = s.charAt(i++);
                if (c=='"') return sb.toString();
                if (c=='\\') { char e=s.charAt(i++);
                    switch(e) { case '"': sb.append('"'); break; case '\\': sb.append('\\'); break;
                        case '/': sb.append('/'); break; case 'n': sb.append('\n'); break;
                        case 'r': sb.append('\r'); break; case 't': sb.append('\t'); break;
                        default: sb.append(e); }
                } else sb.append(c);
            }
            throw err("Unterminated string");
        }
        private Boolean parseBool() {
            if (s.startsWith("true", i)) { i+=4; return Boolean.TRUE; }
            if (s.startsWith("false",i)) { i+=5; return Boolean.FALSE; }
            throw err("Bad boolean");
        }
        private Object parseNull() { if (s.startsWith("null",i)) { i+=4; return null; } throw err("Bad null"); }
        private Number parseNum() {
            int st=i; if (peek('-')) i++;
            while (i<s.length()&&Character.isDigit(s.charAt(i))) i++;
            if (peek('.')) { i++; while (i<s.length()&&Character.isDigit(s.charAt(i))) i++; }
            if (peek('e')||peek('E')) { i++; if (peek('+')||peek('-')) i++;
                while (i<s.length()&&Character.isDigit(s.charAt(i))) i++; }
            return Double.valueOf(s.substring(st,i));
        }
        private void skipWs() { while (i<s.length()) { char c=s.charAt(i); if(c==' '||c=='\n'||c=='\r'||c=='\t') i++; else break; } }
        private void expect(char c) { skipWs(); if (i>=s.length()||s.charAt(i)!=c) throw err("Expected '"+c+"'"); i++; }
        private boolean peek(char c) { return i<s.length()&&s.charAt(i)==c; }
        private RuntimeException err(String m) { return new RuntimeException(m+" at "+i); }
    }
}