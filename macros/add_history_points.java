package macro;

import java.io.*;
import java.lang.reflect.*;
import java.util.*;

import star.common.*;
import star.base.neo.*;
import star.base.report.*;
import star.vis.*;

/**
 * add_history_points.java
 *
 * Creates one PointPart + two MaxReport (Vx, Vy) + two ReportMonitor per
 * entry in data/observations_list.txt.  Monitors update every timestep.
 *
 * Run once after building the mesh.  Save the .sim file when done.
 * The realtime macro reads report values at runtime to build observations.txt.
 *
 * Config keys used (config/case_config.json):
 *   probes.points_file   path to XYZ list   (default: data/observations_list.txt)
 *   probes.region        region name         (default: fluid)
 *
 * Probe file format (one probe per line, whitespace-separated):
 *   x  y  z  [name]
 * Lines starting with # or blank are ignored.
 */
public class add_history_points extends StarMacro {

    @Override
    public void execute() {
        Simulation sim = getActiveSimulation();

        /* ── 1. Resolve repo root ────────────────────────────────────────── */
        File repoRoot = null;
        String sessionPath = sim.getSessionPath();
        if (sessionPath != null && !sessionPath.trim().isEmpty()) {
            File simDir = new File(sessionPath).getParentFile();
            repoRoot = (simDir != null) ? simDir.getParentFile() : null;
        }
        log(sim, "Repo root   : " + repoRoot);

        /* ── 2. Config ───────────────────────────────────────────────────── */
        String cfgPath = System.getenv("STARCCM_JSON");
        if (cfgPath == null || cfgPath.trim().isEmpty()) {
            if (repoRoot != null)
                cfgPath = new File(repoRoot, "config/case_config.json").getAbsolutePath();
            if (cfgPath == null || cfgPath.trim().isEmpty())
                cfgPath = "config/case_config.json";
        }
        log(sim, "Config      : " + cfgPath);

        JsonObject cfg     = Json.parseFile(cfgPath);
        String pointsFile  = resolvePath(cfg.getString("probes.points_file", "data/observations_list.txt"), repoRoot);

        log(sim, "Probes file : " + pointsFile);

        /* ── 3. Parse probe positions ────────────────────────────────────── */
        List<double[]> coords = new ArrayList<double[]>();
        List<String>   names  = new ArrayList<String>();

        try (BufferedReader br = new BufferedReader(new FileReader(pointsFile))) {
            String line;
            int lineNum = 0, probeIdx = 1;
            while ((line = br.readLine()) != null) {
                lineNum++;
                line = line.trim();
                if (line.isEmpty() || line.startsWith("#") || line.startsWith("//")) continue;
                String[] tok = line.split("\\s+");
                if (tok.length < 3) {
                    sim.println("[add_history_points] WARNING line " + lineNum
                        + " skipped (need x y z): " + line);
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
                    sim.println("[add_history_points] WARNING line " + lineNum
                        + " parse error: " + line);
                }
            }
        } catch (IOException ex) {
            throw new RuntimeException("[add_history_points] Cannot read: " + pointsFile, ex);
        }

        if (coords.isEmpty())
            throw new RuntimeException("[add_history_points] No valid probes in: " + pointsFile);
        log(sim, "Loaded " + coords.size() + " probe positions.");

        /* ── 4. Field functions ──────────────────────────────────────────── */
        Units m = sim.getUnitsManager().getUnits("m");

        PrimitiveFieldFunction velocity = (PrimitiveFieldFunction)
            sim.getFieldFunctionManager().getFunction("Velocity");
        FieldFunction ffVx = velocity.getComponentFunction(0);
        FieldFunction ffVy = velocity.getComponentFunction(1);

        log(sim, "Field fns   : " + ffVx.getPresentationName()
            + ", " + ffVy.getPresentationName());

        /* ── 5. Remove existing parts / reports / monitors ───────────────── */
        for (String name : names) {
            for (String comp : new String[]{"Vx", "Vy"}) {
                safeRemove(sim.getMonitorManager(), "Mon_" + name + "_" + comp, sim);
                safeRemove(sim.getReportManager(),  "Rep_" + name + "_" + comp, sim);
            }
            safeRemove(sim.getPartManager(), "Part_" + name, sim);
        }

        /* ── 6. Create probes ────────────────────────────────────────────── */
        int created = 0;
        for (int i = 0; i < coords.size(); i++) {
            double[] xyz  = coords.get(i);
            String   name = names.get(i);

            try {
                /* Point derived part */
                PointPart pt = sim.getPartManager().createPointPart();
                pt.setPresentationName("Part_" + name);
                pt.getPointCoordinate().setCoordinate(m, m, m,
                    new DoubleVector(new double[]{xyz[0], xyz[1], xyz[2]}));
                pt.getInputParts().setObjects(sim.getRegionManager().getObjects());

                /* Vx and Vy MaxReport + ReportMonitor */
                for (int j = 0; j < 2; j++) {
                    FieldFunction ff   = (j == 0) ? ffVx : ffVy;
                    String        comp = (j == 0) ? "Vx"  : "Vy";

                    MaxReport rep = (MaxReport) sim.getReportManager()
                        .createReport(MaxReport.class);
                    rep.setPresentationName("Rep_" + name + "_" + comp);
                    rep.setFieldFunction(ff);
                    rep.getParts().setObjects(pt);

                    ReportMonitor mon = rep.createMonitor();
                    mon.setPresentationName("Mon_" + name + "_" + comp);

                    StarUpdate su = mon.getStarUpdate();
                    su.getUpdateModeOption().setSelected(StarUpdateModeOption.Type.TIMESTEP);
                    su.getTimeStepUpdateFrequency().setTimeSteps(1);
                }

                log(sim, String.format(java.util.Locale.US,
                    "  %-22s  (%.4g, %.4g, %.4g)", name, xyz[0], xyz[1], xyz[2]));
                created++;

            } catch (Exception ex) {
                sim.println("[add_history_points] ERROR creating '" + name
                    + "': " + ex.getMessage());
            }
        }

        log(sim, "Created " + created + " / " + coords.size() + " probes.");
        log(sim, "Total observation values: " + (created * 2) + "  (Vx + Vy per probe).");
        log(sim, ">>> Save the .sim file before running the realtime macro. <<<");
    }

    /* ── helpers ─────────────────────────────────────────────────────────── */

    /** Remove a named object from any manager using reflection — silent on failure. */
    @SuppressWarnings({"unchecked", "rawtypes"})
    private static void safeRemove(Object manager, String name, Simulation sim) {
        try {
            Object obj = null;
            for (String getter : new String[]{"getMonitor", "getReport", "getPart", "getObject"}) {
                try {
                    Method g = manager.getClass().getMethod(getter, String.class);
                    obj = g.invoke(manager, name);
                    if (obj != null) break;
                } catch (Exception ignored) {}
            }
            if (obj == null) return;
            ArrayList list = new ArrayList();
            list.add(obj);
            for (Method rm : manager.getClass().getMethods()) {
                if (!rm.getName().equals("removeObjects") || rm.getParameterCount() != 1) continue;
                try { rm.invoke(manager, list); sim.println("[add_history_points] Removed: " + name); return; }
                catch (Exception ignored) {}
            }
        } catch (Exception ignored) {}
    }

    private static String resolvePath(String path, File repoRoot) {
        if (path == null || path.isEmpty() || new File(path).isAbsolute() || repoRoot == null)
            return path;
        return new File(repoRoot, path).getAbsolutePath();
    }

    private static void log(Simulation s, String m) { s.println("[add_history_points] " + m); }

    /* ── mini JSON parser ────────────────────────────────────────────────── */

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
        String getString(String path, String d) {
            Object v = get(path);
            return (v instanceof String) ? (String) v : d;
        }
        private Object get(String path) {
            String[] parts = path.split("\\.");
            Object cur = map;
            for (String p : parts) {
                if (!(cur instanceof Map)) return null;
                cur = ((Map<?, ?>) cur).get(p);
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
            if (c == '{') return parseObj();
            if (c == '[') return parseArr();
            if (c == '"') return parseStr();
            if (c == 't' || c == 'f') return parseBool();
            if (c == 'n') return parseNull();
            return parseNum();
        }
        private Map<String, Object> parseObj() {
            expect('{');
            LinkedHashMap<String, Object> out = new LinkedHashMap<String, Object>();
            skipWs(); if (peek('}')) { i++; return out; }
            while (true) {
                skipWs(); String k = parseStr(); skipWs(); expect(':');
                out.put(k, parseValue()); skipWs();
                if (peek('}')) { i++; break; } expect(',');
            }
            return out;
        }
        private List<Object> parseArr() {
            expect('['); List<Object> out = new ArrayList<Object>();
            skipWs(); if (peek(']')) { i++; return out; }
            while (true) {
                out.add(parseValue()); skipWs();
                if (peek(']')) { i++; break; } expect(',');
            }
            return out;
        }
        private String parseStr() {
            expect('"'); StringBuilder sb = new StringBuilder();
            while (i < s.length()) {
                char c = s.charAt(i++);
                if (c == '"') return sb.toString();
                if (c == '\\') {
                    char e = s.charAt(i++);
                    switch (e) {
                        case '"':  sb.append('"');  break;
                        case '\\': sb.append('\\'); break;
                        case '/':  sb.append('/');  break;
                        case 'n':  sb.append('\n'); break;
                        case 'r':  sb.append('\r'); break;
                        case 't':  sb.append('\t'); break;
                        default:   sb.append(e);
                    }
                } else sb.append(c);
            }
            throw err("Unterminated string");
        }
        private Boolean parseBool() {
            if (s.startsWith("true",  i)) { i += 4; return Boolean.TRUE; }
            if (s.startsWith("false", i)) { i += 5; return Boolean.FALSE; }
            throw err("Bad boolean");
        }
        private Object parseNull() {
            if (s.startsWith("null", i)) { i += 4; return null; }
            throw err("Bad null");
        }
        private Number parseNum() {
            int st = i; if (peek('-')) i++;
            while (i < s.length() && Character.isDigit(s.charAt(i))) i++;
            if (peek('.')) { i++; while (i < s.length() && Character.isDigit(s.charAt(i))) i++; }
            if (peek('e') || peek('E')) {
                i++; if (peek('+') || peek('-')) i++;
                while (i < s.length() && Character.isDigit(s.charAt(i))) i++;
            }
            return Double.valueOf(s.substring(st, i));
        }
        private void skipWs() {
            while (i < s.length()) {
                char c = s.charAt(i);
                if (c == ' ' || c == '\n' || c == '\r' || c == '\t') i++; else break;
            }
        }
        private void expect(char c) {
            skipWs();
            if (i >= s.length() || s.charAt(i) != c) throw err("Expected '" + c + "'");
            i++;
        }
        private boolean peek(char c) { return i < s.length() && s.charAt(i) == c; }
        private RuntimeException err(String m) { return new RuntimeException(m + " at " + i); }
    }
}
