package macro;

import java.io.*;
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
 *   for each timestep:
 *     1. wait for agent to write action  (polls ready_flag)
 *     2. read action, apply BC
 *     3. advance exactly ONE timestep    (MaxInnerIterations = N, MaxSteps += 1)
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
        String cfgPath = System.getenv("STARCCM_JSON");
        if (cfgPath == null || cfgPath.trim().isEmpty())
            cfgPath = "F:/STARCCM-UMICH/Nueva Carpeta/project-cylAFC/case_config.JSON";

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
            runRealtime(sim, topJet, botJet, cfg);
        } else if ("table".equals(mode)) {
            setupTable(sim, topJet, botJet, cfg);
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
            Boundary topJet, Boundary botJet, JsonObject cfg) {

        String path = cfg.getString("jets.table_path", "");
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
            Boundary topJet, Boundary botJet, JsonObject cfg) {

        String actionFile = cfg.getString("jets.realtime.action_file", "jet_action.txt");
        String readyFlag  = cfg.getString("jets.realtime.ready_flag",  "agent_ready.flag");
        String obsFlag    = cfg.getString("jets.realtime.obs_flag",    "starccm_ready.flag");
        long   pollMs     = (long) cfg.getDouble("jets.realtime.poll_interval_ms", 50.0);
        long   timeoutMs  = (long)(cfg.getDouble("jets.realtime.poll_timeout_s",   60.0) * 1000.0);
        int    nSteps     = cfg.getInt("time.steps", 100);

        log(sim, "Realtime loop: " + nSteps + " timesteps.");
        log(sim, "  action_file : " + actionFile);
        log(sim, "  ready_flag  : " + readyFlag);
        log(sim, "  obs_flag    : " + obsFlag);

        // Get the MaxSteps stopping criterion — we advance it by 1 each iteration.
        StepStoppingCriterion maxSteps = getMaxStepsCriterion(sim);
        int currentStepLimit = maxSteps.getMaximumNumberSteps();

        // Signal agent: ready for first action.
        touchFile(obsFlag, sim);

        double lastV = 0.0;

        for (int step = 0; step < nSteps; step++) {

            log(sim, "--- Step " + (step + 1) + " / " + nSteps + " ---");

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

            // 3. Advance exactly one timestep by incrementing the step limit by 1
            currentStepLimit += 1;
            maxSteps.setMaximumNumberSteps(currentStepLimit);
            sim.getSimulationIterator().run();

            // 4. Signal agent: step done, observations available
            touchFile(obsFlag, sim);
            log(sim, "Step done — obs_flag written.");
        }

        log(sim, "Realtime loop complete.");
    }

    /*------------------------------------------------------------*/
    /*  Helpers for realtime                                      */
    /*------------------------------------------------------------*/
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

    private static StepStoppingCriterion getMaxStepsCriterion(Simulation sim) {
        return (StepStoppingCriterion) sim.getSolverStoppingCriterionManager()
                .getSolverStoppingCriterion("Maximum Steps");
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