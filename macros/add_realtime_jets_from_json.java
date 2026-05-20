package macro;

import java.io.*;
import java.lang.reflect.*;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

import star.common.*;
import star.flow.*;

/**
 * add_realtime_jets_from_json.java
 *
 * Reads case_config.json and sets jet boundary conditions.
 * Supported modes (jets.mode in JSON):
 *   - "prescribed" : antisymmetric sinusoid  A sin(2*pi*f*t)
 *   - "table"      : piecewise-constant schedule from text file
 *   - "realtime"   : live file-polling, one action per timestep
 *
 * Realtime strategy: instead of registering a listener (no stable
 * public listener API in STAR-CCM+ 2602), the macro takes full control
 * of the run loop itself:
 *
 *   for each agent step (= action_repeat physical timesteps):
 *     1. wait for agent to write action  (polls ready_flag)
 *     2. read action, apply BC
 *     3. advance action_repeat timesteps (MaxPhysicalTime += action_repeat * dt)
 *     4. write obs_flag so agent can read observations
 *     5. repeat
 *
 * This avoids any listener/observer API entirely.
 * The macro blocks in Java while the solver runs each step.
 *
 * STAR-CCM+ version target: 2602 (2026-02)
 */
public class add_realtime_jets_from_json extends StarMacro {

    @Override
    public void execute() {

        Simulation sim = getActiveSimulation();

        /* 1. read JSON ---------------------------------------------------- */
        File repoRoot = null;
        String sessionPath = sim.getSessionPath();
        if (sessionPath != null && !sessionPath.trim().isEmpty()) {
            File simDir = new File(sessionPath).getParentFile();
            repoRoot = (simDir != null) ? simDir.getParentFile() : null;
        }

        String cfgPath = System.getenv("STARCCM_JSON");
        if (cfgPath == null || cfgPath.trim().isEmpty()) {
            if (repoRoot != null)
                cfgPath = new File(repoRoot, "config/case_config.json").getAbsolutePath();
            if (cfgPath == null || cfgPath.trim().isEmpty())
                cfgPath = "config/case_config.json";
        }

        JsonObject cfg = Json.parseFile(cfgPath);
        String mode = cfg.getString("jets.mode", "prescribed").toLowerCase();
        log(sim, "jets.mode = " + mode);

        /* 2. locate boundaries ------------------------------------------- */
        Region   fluid  = sim.getRegionManager().getRegion("fluid");
        Boundary topJet = fluid.getBoundaryManager().getBoundary("jet_top");
        Boundary botJet = fluid.getBoundaryManager().getBoundary("jet_bot");

        cleanJetBoundariesAndFunctions(sim, fluid);

        /* 3. mode dispatch ------------------------------------------------ */
        if ("realtime".equals(mode)) {
            runRealtime(sim, topJet, botJet, cfg, repoRoot);
        } else if ("table".equals(mode)) {
            setupTable(sim, topJet, botJet, cfg, repoRoot);
        } else {
            setupPrescribed(sim, topJet, botJet, cfg);
        }
    }

    /*------------------------------------------------------------*/
    /*  PRESCRIBED                                                */
    /*------------------------------------------------------------*/
    private static void setupPrescribed(Simulation sim,
            Boundary topJet, Boundary botJet, JsonObject cfg) {

        double amp  = cfg.getDouble("jets.amplitude", 0.1);
        double freq = cfg.getDouble("jets.frequency", 0.16);
        String topExpr = String.format(java.util.Locale.US,
                "%s*sin(2*3.14159265358979323846*%s*$Time)", fmt(amp), fmt(freq));
        String botExpr = "-(" + topExpr + ")";

        assignFunctionToBoundary(topJet, createScalarVelocityFunction(sim, "jet_top_func", topExpr));
        assignFunctionToBoundary(botJet, createScalarVelocityFunction(sim, "jet_bot_func", botExpr));
        log(sim, "Prescribed sinusoid applied (A=" + amp + ", f=" + freq + " Hz).");
    }

    /*------------------------------------------------------------*/
    /*  TABLE                                                     */
    /*------------------------------------------------------------*/
    private static void setupTable(Simulation sim,
            Boundary topJet, Boundary botJet, JsonObject cfg, File repoRoot) {

        String path = resolvePath(cfg.getString("jets.table_path", ""), repoRoot);
        if (path.isEmpty()) throw new RuntimeException("jets.table_path must be set for table mode.");
        double dt = cfg.getDouble("jets.delta_t_action", 0.0);
        if (dt <= 0) throw new RuntimeException("jets.delta_t_action must be > 0 for table mode.");

        double[] v = readTableFile(path);
        assignFunctionToBoundary(topJet, createScalarVelocityFunction(sim, "jet_top_func", buildPiecewiseExpr(v, dt, false)));
        assignFunctionToBoundary(botJet, createScalarVelocityFunction(sim, "jet_bot_func", buildPiecewiseExpr(v, dt, true)));
        log(sim, "Table mode applied (" + v.length + " actions, dt=" + dt + " s).");
    }

    /*------------------------------------------------------------*/
    /*  REALTIME — macro-controlled run loop                      */
    /*------------------------------------------------------------*/
    private static void runRealtime(Simulation sim,
            Boundary topJet, Boundary botJet, JsonObject cfg, File repoRoot) {

        String actionFile = resolvePath(cfg.getString("jets.realtime.action_file", "runtime/jet_action.txt"),       repoRoot);
        String readyFlag  = resolvePath(cfg.getString("jets.realtime.ready_flag",  "runtime/agent_ready.flag"),     repoRoot);
        String obsFlag    = resolvePath(cfg.getString("jets.realtime.obs_flag",    "runtime/starccm_ready.flag"),   repoRoot);
        String obsFile    = resolvePath(cfg.getString("jets.realtime.obs_file",    "runtime/observations.txt"),     repoRoot);
        String forcesFile = resolvePath(cfg.getString("jets.realtime.drag_file",   "runtime/forces.txt"),           repoRoot);
        String pointsFile = resolvePath(cfg.getString("probes.points_file", "data/observations_list.txt"),          repoRoot);
        long   pollMs      = (long) cfg.getDouble("jets.realtime.poll_interval_ms", 50.0);
        long   timeoutMs   = (long)(cfg.getDouble("jets.realtime.poll_timeout_s",   60.0) * 1000.0);
        int    nSteps      = cfg.getInt("jets.realtime.n_steps", 1000);
        double dt          = cfg.getDouble("time.dt", 0.01);
        int    actionRepeat = cfg.getInt("jets.realtime.action_repeat", 1);
        int    agentSteps  = nSteps / actionRepeat;
        double agentDt     = actionRepeat * dt;

        disableAutosave(sim);

        List<String> probeNames = loadProbeNames(sim, pointsFile);
        log(sim, "Realtime loop: " + nSteps + " CFD timesteps, dt=" + dt + " s, "
                + actionRepeat + " timesteps/action (" + agentDt + " s per action), "
                + agentSteps + " agent steps, "
                + (probeNames.size() * 2) + " obs values.");
        log(sim, "  action_file : " + actionFile);
        log(sim, "  ready_flag  : " + readyFlag);
        log(sim, "  obs_flag    : " + obsFlag);
        log(sim, "  obs_file    : " + obsFile);

        PhysicalTimeStoppingCriterion timeCrit = getPhysicalTimeCriterion(sim);

        // Signal agent: ready for first action.
        touchFile(obsFlag, sim);

        double lastV = 0.0;

        for (int step = 0; step < agentSteps; step++) {

            double tStart = sim.getSolution().getPhysicalTime();
            double tEnd   = tStart + agentDt;
            log(sim, String.format(java.util.Locale.US,
                "--- Agent step %d / %d  (t=%.4g → %.4g s) ---",
                step + 1, agentSteps, tStart, tEnd));

            // 1. Wait for agent to provide action
            log(sim, "Waiting for agent_ready.flag ...");
            double v = waitForAction(sim, actionFile, readyFlag, pollMs, timeoutMs, lastV);
            deleteFile(readyFlag);
            lastV = v;

            // 2. Apply BC
            setConstVel(topJet,  v);
            setConstVel(botJet, -v);
            log(sim, String.format(java.util.Locale.US,
                "Applied: top=%.6g  bot=%.6g", v, -v));

            // 3. Advance exactly actionRepeat physical timesteps (unambiguous: no iteration confusion)
            timeCrit.getMaximumTime().setValue(tEnd);
            sim.getSimulationIterator().run();

            // 4. Write observations from the last sub-step, then signal agent
            writeObservations(sim, probeNames, obsFile, forcesFile);
            touchFile(obsFlag, sim);
            log(sim, "Agent step done — obs written, obs_flag touched.");
        }

        log(sim, "Realtime loop complete.");
    }

    /*------------------------------------------------------------*/
    /*  Helpers for realtime                                      */
    /*------------------------------------------------------------*/

    /** Disable STAR-CCM+ autosave so it doesn't block the realtime loop. */
    private static void disableAutosave(Simulation sim) {
        // Try the AutoSave object accessible from the sim or its iterator
        for (String getter : new String[]{
                "getAutoSaveManager", "getAutosaveManager", "getAutoSave"}) {
            try {
                Object asm = sim.getClass().getMethod(getter).invoke(sim);
                if (asm == null) continue;
                boolean acted = false;
                for (Method m : asm.getClass().getMethods()) {
                    if (m.getParameterCount() != 1) continue;
                    String n = m.getName();
                    if (m.getParameterTypes()[0] == boolean.class &&
                            (n.equals("setEnabled") || n.equals("setActive") || n.equals("setAutoSaveEnabled"))) {
                        m.invoke(asm, Boolean.FALSE);
                        log(sim, "Autosave disabled via " + getter + "()." + n + "(false).");
                        acted = true;
                    }
                }
                if (acted) return;
            } catch (Exception ignored) {}
        }
        // Fallback: try sim.getSimulationIterator() path
        try {
            Object iter = sim.getClass().getMethod("getSimulationIterator").invoke(sim);
            for (String getter : new String[]{"getAutoSave", "getAutoSaveManager"}) {
                try {
                    Object asm = iter.getClass().getMethod(getter).invoke(iter);
                    if (asm == null) continue;
                    for (Method m : asm.getClass().getMethods()) {
                        if (m.getParameterCount() == 1 &&
                                m.getParameterTypes()[0] == boolean.class &&
                                m.getName().startsWith("set")) {
                            String n = m.getName().toLowerCase();
                            if (n.contains("enable") || n.contains("active")) {
                                m.invoke(asm, Boolean.FALSE);
                                log(sim, "Autosave disabled via iterator." + getter + "()." + m.getName() + "(false).");
                                return;
                            }
                        }
                    }
                } catch (Exception ignored) {}
            }
        } catch (Exception ignored) {}
        log(sim, "WARNING: could not disable autosave programmatically. " +
                "Disable it manually: Tools → Options → Environment → Autosave.");
    }

    private static double waitForAction(Simulation sim,
            String actionFile, String readyFlag,
            long pollMs, long timeoutMs, double lastV) {

        long deadline = System.currentTimeMillis() + timeoutMs;
        while (System.currentTimeMillis() < deadline) {
            if (new File(readyFlag).exists()) {
                return readAction(sim, actionFile, lastV);
            }
            try { Thread.sleep(pollMs); }
            catch (InterruptedException ex) { Thread.currentThread().interrupt(); break; }
        }
        sim.println(String.format(java.util.Locale.US,
            "[jets-rt] WARNING: timeout — holding last velocity %.6g m/s", lastV));
        return lastV;
    }

    private static double readAction(Simulation sim, String actionFile, double lastV) {
        try (BufferedReader br = new BufferedReader(new FileReader(actionFile))) {
            String line;
            while ((line = br.readLine()) != null) {
                line = line.trim();
                if (line.isEmpty() || line.startsWith("#") || line.startsWith("//")) continue;
                double v = Double.parseDouble(line);
                sim.println(String.format(java.util.Locale.US,
                    "[jets-rt] Action received: %.6g m/s", v));
                return v;
            }
        } catch (Exception ex) {
            sim.println("[jets-rt] ERROR reading action file: " + ex.getMessage());
        }
        return lastV;
    }

    private static PhysicalTimeStoppingCriterion getPhysicalTimeCriterion(Simulation sim) {
        return (PhysicalTimeStoppingCriterion) sim.getSolverStoppingCriterionManager()
                .getSolverStoppingCriterion("Maximum Physical Time");
    }

    private static void setConstVel(Boundary b, double v) {
        VelocityMagnitudeProfile p = b.getValues().get(VelocityMagnitudeProfile.class);
        p.setMethod(ConstantScalarProfileMethod.class);
        p.getMethod(ConstantScalarProfileMethod.class).getQuantity().setValue(v);
    }

    /*######################################################################*/
    /*  SHARED UTILS                                                       */
    /*######################################################################*/
    private static String buildPiecewiseExpr(double[] v, double dt, boolean neg) {
        StringBuilder sb = new StringBuilder();
        for (int i = 0; i < v.length - 1; i++) {
            double edge = (i + 1) * dt;
            double val  = neg ? -v[i] : v[i];
            sb.append(String.format(java.util.Locale.US, "($Time<%.17g)?%.17g:", edge, val));
        }
        double last = neg ? -v[v.length - 1] : v[v.length - 1];
        sb.append(String.format(java.util.Locale.US, "%.17g", last));
        return sb.toString();
    }

    private static double[] readTableFile(String path) {
        List<Double> list = new ArrayList<Double>();
        try (BufferedReader br = new BufferedReader(new FileReader(path))) {
            String line;
            while ((line = br.readLine()) != null) {
                line = line.trim();
                if (line.isEmpty() || line.startsWith("#") || line.startsWith("//")) continue;
                list.add(Double.parseDouble(line));
            }
        } catch (Exception ex) { throw new RuntimeException("Error reading table: " + ex.getMessage()); }
        if (list.isEmpty()) throw new RuntimeException("Jet table file is empty.");
        double[] arr = new double[list.size()];
        for (int i = 0; i < arr.length; i++) arr[i] = list.get(i);
        return arr;
    }

    private static UserFieldFunction createScalarVelocityFunction(
            Simulation sim, String name, String def) {
        List<UserFieldFunction> rm = new ArrayList<UserFieldFunction>();
        for (FieldFunction f : sim.getFieldFunctionManager().getObjects()) {
            if (!(f instanceof UserFieldFunction)) continue;
            UserFieldFunction uff = (UserFieldFunction) f;
            if (name.equals(uff.getFunctionName()) || name.equals(uff.getPresentationName()))
                rm.add(uff);
        }
        if (!rm.isEmpty()) sim.getFieldFunctionManager().removeObjects(rm);
        UserFieldFunction ff = sim.getFieldFunctionManager().createFieldFunction();
        ff.setPresentationName(name);
        ff.setFunctionName(name);
        ff.getTypeOption().setSelected(FieldFunctionTypeOption.Type.SCALAR);
        ff.setDimensions(Dimensions.Builder().velocity(1).build());
        ff.setDefinition(def);
        return ff;
    }

    private static void assignFunctionToBoundary(Boundary b, UserFieldFunction ff) {
        VelocityMagnitudeProfile p = b.getValues().get(VelocityMagnitudeProfile.class);
        p.setMethod(FunctionScalarProfileMethod.class);
        p.getMethod(FunctionScalarProfileMethod.class).setFieldFunction(ff);
    }

    private static void cleanJetBoundariesAndFunctions(Simulation sim, Region fluid) {
        for (String n : new String[]{"jet_top", "jet_bot"}) {
            try {
                Boundary b = fluid.getBoundaryManager().getBoundary(n);
                VelocityMagnitudeProfile p = b.getValues().get(VelocityMagnitudeProfile.class);
                p.setMethod(ConstantScalarProfileMethod.class);
                p.getMethod(ConstantScalarProfileMethod.class).getQuantity().setValue(0.0);
            } catch (Exception ex) {
                sim.println("[jets_macro] Could not reset boundary " + n + ": " + ex.getMessage());
            }
        }
        for (String n : new String[]{"jet_top_func", "jet_bot_func"}) {
            List<UserFieldFunction> rm = new ArrayList<UserFieldFunction>();
            for (FieldFunction f : sim.getFieldFunctionManager().getObjects()) {
                if (!(f instanceof UserFieldFunction)) continue;
                UserFieldFunction uff = (UserFieldFunction) f;
                if (n.equals(uff.getFunctionName()) || n.equals(uff.getPresentationName()))
                    rm.add(uff);
            }
            if (!rm.isEmpty()) {
                for (UserFieldFunction uff : rm)
                    sim.println("[jets_macro] Removing stale function: " + uff.getPresentationName());
                sim.getFieldFunctionManager().removeObjects(rm);
            }
        }
    }

    /** Load probe names (4th column) from observations_list.txt. */
    private static List<String> loadProbeNames(Simulation sim, String pointsFile) {
        List<String> names = new ArrayList<String>();
        int idx = 1;
        try (BufferedReader br = new BufferedReader(new FileReader(pointsFile))) {
            String line;
            while ((line = br.readLine()) != null) {
                line = line.trim();
                if (line.isEmpty() || line.startsWith("#") || line.startsWith("//")) continue;
                String[] tok = line.split("\\s+");
                if (tok.length < 3) continue;
                names.add(tok.length >= 4 ? tok[3] : ("probe_" + idx));
                idx++;
            }
        } catch (Exception ex) {
            sim.println("[jets-rt] WARN: cannot load probe names: " + ex.getMessage());
        }
        log(sim, "Loaded " + names.size() + " probe names from " + pointsFile);
        return names;
    }

    /**
     * Read Vx and Vy from each probe's MaxReport and write space-separated
     * values to obsFile and forcesFile.  Uses reflection to call getValue()
     * so the code is robust across STAR-CCM+ API variations.
     */
    private static void writeObservations(Simulation sim, List<String> probeNames,
            String obsFile, String forcesFile) {
        StringBuilder sb = new StringBuilder();
        for (String name : probeNames) {
            for (String comp : new String[]{"Vx", "Vy"}) {
                if (sb.length() > 0) sb.append(" ");
                String repName = "Rep_" + name + "_" + comp;
                double val = 0.0;
                try {
                    Object rep = sim.getReportManager().getReport(repName);
                    val = evalReport(rep);
                } catch (Exception ex) {
                    sim.println("[jets-rt] WARN: cannot read " + repName + ": " + ex.getMessage());
                }
                sb.append(String.format(java.util.Locale.US, "%.10g", val));
            }
        }
        String content = sb.toString();
        writeTextFile(obsFile,    content, sim);
        writeTextFile(forcesFile, content, sim);
    }

    /** Try several reflection-based approaches to get a scalar double from a Report object. */
    private static double evalReport(Object rep) {
        // 1. getReportMonitorValue() → last monitor-recorded value
        try {
            Method m = rep.getClass().getMethod("getReportMonitorValue");
            Object rv = m.invoke(rep);
            if (rv instanceof Number) return ((Number) rv).doubleValue();
        } catch (Exception ignored) {}
        // 2. getValue(Units=null) → evaluate on current field, base SI units
        try {
            for (Method m : rep.getClass().getMethods()) {
                if (!m.getName().equals("getValue") || m.getParameterCount() != 1) continue;
                try {
                    Object rv = m.invoke(rep, (Object) null);
                    if (rv instanceof Number) return ((Number) rv).doubleValue();
                } catch (Exception ignored2) {}
            }
        } catch (Exception ignored) {}
        // 3. getValue() no-arg
        try {
            Method m = rep.getClass().getMethod("getValue");
            Object rv = m.invoke(rep);
            if (rv instanceof Number) return ((Number) rv).doubleValue();
        } catch (Exception ignored) {}
        return Double.NaN;
    }

    private static void writeTextFile(String path, String content, Simulation sim) {
        try {
            File f = new File(path);
            if (f.getParentFile() != null) f.getParentFile().mkdirs();
            try (PrintWriter pw = new PrintWriter(new FileWriter(f, false))) {
                pw.println(content);
            }
        } catch (IOException ex) {
            sim.println("[jets-rt] WARN write " + path + ": " + ex.getMessage());
        }
    }

    private static void touchFile(String p, Simulation sim) {
        try {
            File f = new File(p);
            if (f.getParentFile() != null) f.getParentFile().mkdirs();
            new FileOutputStream(f, false).close();
        } catch (IOException ex) {
            sim.println("[jets-rt] WARN touch " + p + ": " + ex.getMessage());
        }
    }

    private static void deleteFile(String p) { new File(p).delete(); }
    private static String resolvePath(String path, File repoRoot) {
        if (path == null || path.isEmpty() || new File(path).isAbsolute() || repoRoot == null) return path;
        return new File(repoRoot, path).getAbsolutePath();
    }
    private static void log(Simulation s, String m) { s.println("[jets_macro] " + m); }
    private static String fmt(double v) { return String.format(java.util.Locale.US, "%.16g", v); }

    /*######################################################################*/
    /*  Mini JSON parser                                                   */
    /*######################################################################*/
    static final class Json {
        static JsonObject parseFile(String path) {
            StringBuilder sb = new StringBuilder();
            try (BufferedReader br = new BufferedReader(new FileReader(path))) {
                String line;
                while ((line = br.readLine()) != null) sb.append(line).append('\n');
            } catch (IOException ex) { throw new RuntimeException("Cannot read JSON: " + path, ex); }
            Object parsed = new Parser(sb.toString()).parseValue();
            if (!(parsed instanceof Map)) throw new RuntimeException("Top-level JSON must be an object.");
            @SuppressWarnings("unchecked")
            JsonObject obj = new JsonObject((Map<String, Object>) parsed);
            return obj;
        }
    }

    static final class JsonObject {
        private final Map<String, Object> map;
        JsonObject(Map<String, Object> m) { this.map = m; }
        double getDouble(String path, double d) { Object v = get(path); return (v instanceof Number) ? ((Number)v).doubleValue() : d; }
        int    getInt   (String path, int    d) { Object v = get(path); return (v instanceof Number) ? ((Number)v).intValue()    : d; }
        String getString(String path, String d) { Object v = get(path); return (v instanceof String) ? (String)v                 : d; }
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
                        case '/': sb.append('/'); break; case 'b': sb.append('\b'); break;
                        case 'f': sb.append('\f'); break; case 'n': sb.append('\n'); break;
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