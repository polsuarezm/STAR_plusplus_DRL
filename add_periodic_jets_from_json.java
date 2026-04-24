package macro;

import java.io.BufferedReader;
import java.io.FileReader;
import java.io.IOException;
import java.lang.reflect.Method;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

import star.base.neo.*;
import star.common.*;
import star.flow.*;

/**
 * cylinder_afc_set_jets.java
 *
 * Standalone macro that reads case_config.json and sets the jet boundary
 * conditions on an already-open STAR-CCM+ simulation.
 *
 * Two modes, selected by jets.mode in the JSON:
 *
 *   "prescribed"  – antisymmetric sinusoid
 *                   top:  A * sin(2*pi*freq*$Time)
 *                   bot: -A * sin(2*pi*freq*$Time)
 *
 *   "table"       – piecewise-constant schedule read from a plain text file.
 *                   The file (one value per line) contains N velocity values,
 *                   one per action interval of length delta_t_action [s].
 *                   Top jet takes the value directly; bottom jet is negated
 *                   (antiphase convention, consistent with prescribed mode).
 *                   The last value is held for all t beyond the table range.
 *
 * Both modes write a UserFieldFunction and assign it via
 * FunctionScalarProfileMethod so the boundary type (velocity-inlet) is
 * unchanged and the setup is easy to inspect / override inside the GUI.
 *
 * STAR-CCM+ version target: 2602 (2026.02)
 */
public class add_periodic_jets_from_json extends StarMacro {

    @Override
    public void execute() {
        Simulation sim = getActiveSimulation();

        // ------------------------------------------------------------------ //
        // 1. Load JSON config
        // ------------------------------------------------------------------ //
        String configPath = System.getenv("STARCCM_JSON");
        if (configPath == null || configPath.trim().isEmpty()) {
            configPath = "F:/STARCCM-UMICH/Nueva Carpeta/project-cylAFC/case_config.JSON";
        }
        log(sim, "Reading config from: " + configPath);
        JsonObject cfg = Json.parseFile(configPath);

        String mode = cfg.getString("jets.mode", "prescribed");
        log(sim, "Jet mode: " + mode);

        // ------------------------------------------------------------------ //
        // 2. Locate boundaries
        // ------------------------------------------------------------------ //
        Region fluid = sim.getRegionManager().getRegion("fluid");
        Boundary topJet = fluid.getBoundaryManager().getBoundary("jet_top");
        Boundary botJet = fluid.getBoundaryManager().getBoundary("jet_bot");

        // ------------------------------------------------------------------ //
        // 3. Build field function expressions
        // ------------------------------------------------------------------ //
        String topExpr;
        String botExpr;

        if ("table".equalsIgnoreCase(mode)) {
            // ---- TABLE mode ----
            String tablePath = cfg.getString("jets.table_path", "");
            if (tablePath.isEmpty()) {
                throw new RuntimeException(
                    "[cylinder_afc_set_jets] jets.table_path must be set when jets.mode = \"table\".");
            }
            double dtAction = cfg.getDouble("jets.delta_t_action", 0.0);
            if (dtAction <= 0.0) {
                throw new RuntimeException(
                    "[cylinder_afc_set_jets] jets.delta_t_action must be > 0 when jets.mode = \"table\".");
            }

            log(sim, "Reading action table from: " + tablePath);
            double[] values = readTableFile(tablePath);
            log(sim, "Table loaded: " + values.length + " actions, delta_t_action = " + dtAction + " s");

            // Build a piecewise-constant expression using nested ternary operators:
            //   ($Time < dt)  ? v0 : ($Time < 2*dt) ? v1 : ... : vN-1
            // The last value is held for all t beyond the table.
            topExpr = buildPiecewiseExpr(values, dtAction, false);
            botExpr = buildPiecewiseExpr(values, dtAction, true);   // negated → antiphase

        } else {
            // ---- PRESCRIBED (sinusoidal) mode ----
            double amp  = cfg.getDouble("jets.amplitude",  0.1);
            double freq = cfg.getDouble("jets.frequency",  0.16);
            log(sim, String.format(java.util.Locale.US,
                "Prescribed sinusoid: A=%.6g, freq=%.6g Hz", amp, freq));

            topExpr = String.format(java.util.Locale.US,
                "%s * sin(2 * 3.14159265358979323846 * %s * $Time)", fmt(amp), fmt(freq));
            botExpr = String.format(java.util.Locale.US,
                "-(%s * sin(2 * 3.14159265358979323846 * %s * $Time))", fmt(amp), fmt(freq));
        }

        log(sim, "top expression : " + topExpr);
        log(sim, "bot expression : " + botExpr);

        // ------------------------------------------------------------------ //
        // 4. Create / update UserFieldFunctions
        // ------------------------------------------------------------------ //
        // Revert boundaries to constant v=0 first so any existing field functions
        // are no longer referenced and can be safely deleted before recreation.
        cleanJetBoundariesAndFunctions(sim);

        UserFieldFunction ffTop = getOrCreateScalarVelocityFunction(sim, "jet_top_func", topExpr);
        UserFieldFunction ffBot = getOrCreateScalarVelocityFunction(sim, "jet_bot_func", botExpr);

        // ------------------------------------------------------------------ //
        // 5. Assign to boundaries via FunctionScalarProfileMethod
        // ------------------------------------------------------------------ //
        assignFunctionToBoundary(topJet, ffTop);
        assignFunctionToBoundary(botJet, ffBot);

        log(sim, "Jet boundary conditions updated successfully.");
    }

    // ======================================================================= //
    // Helper: build piecewise-constant ternary expression from table values
    // ======================================================================= //

    /**
     * Produces a STAR-CCM+ field-function expression of the form:
     *
     *   ($Time < 1*dt) ? v[0] :
     *   ($Time < 2*dt) ? v[1] :
     *   ...
     *   ($Time < N*dt) ? v[N-1] :
     *   v[N-1]                      ← last value held forever
     *
     * If negate=true every value is multiplied by -1 (bottom jet antiphase).
     *
     * STAR-CCM+ ternary syntax:  (condition) ? valueIfTrue : valueIfFalse
     */
    private static String buildPiecewiseExpr(double[] v, double dt, boolean negate) {
        StringBuilder sb = new StringBuilder();
        for (int i = 0; i < v.length - 1; i++) {
            double tEdge = (i + 1) * dt;
            double val   = negate ? -v[i] : v[i];
            sb.append(String.format(java.util.Locale.US,
                "($Time < %.17g) ? %.17g : ", tEdge, val));
        }
        // Final (held) value — no trailing ternary
        double lastVal = negate ? -v[v.length - 1] : v[v.length - 1];
        sb.append(String.format(java.util.Locale.US, "%.17g", lastVal));
        return sb.toString();
    }

    // ======================================================================= //
    // Helper: read plain-text table file (one double per non-empty line)
    // ======================================================================= //

    private static double[] readTableFile(String path) {
        List<Double> list = new ArrayList<Double>();
        try (BufferedReader br = new BufferedReader(new FileReader(path))) {
            String line;
            while ((line = br.readLine()) != null) {
                line = line.trim();
                if (line.isEmpty() || line.startsWith("#") || line.startsWith("//")) continue;
                list.add(Double.parseDouble(line));
            }
        } catch (IOException ex) {
            throw new RuntimeException("Could not read jet table file: " + path, ex);
        } catch (NumberFormatException ex) {
            throw new RuntimeException("Non-numeric entry in jet table file: " + path, ex);
        }
        if (list.isEmpty()) {
            throw new RuntimeException("Jet table file is empty: " + path);
        }
        double[] arr = new double[list.size()];
        for (int i = 0; i < arr.length; i++) arr[i] = list.get(i);
        return arr;
    }

    // ======================================================================= //
    // Helper: revert jet boundaries to constant v=0, then delete stale functions
    // ======================================================================= //

    private static void cleanJetBoundariesAndFunctions(Simulation sim) {
        // Step 1: revert boundaries to ConstantScalarProfileMethod so the
        // field functions are no longer referenced before we delete them.
        String[] boundaryNames = {"jet_top", "jet_bot"};
        try {
            Region fluid = sim.getRegionManager().getRegion("fluid");
            for (String bName : boundaryNames) {
                try {
                    Boundary bnd = fluid.getBoundaryManager().getBoundary(bName);
                    VelocityMagnitudeProfile prof = bnd.getValues().get(VelocityMagnitudeProfile.class);
                    prof.setMethod(ConstantScalarProfileMethod.class);
                    prof.getMethod(ConstantScalarProfileMethod.class).getQuantity().setValue(0.0);
                } catch (Exception ex) {
                    log(sim, "Could not reset boundary " + bName + ": " + ex.getMessage());
                }
            }
        } catch (Exception ex) {
            log(sim, "Could not access fluid region for boundary reset: " + ex.getMessage());
        }

        // Step 2: delete all stale jet field functions (including duplicates)
        String[] funcNames = {"jet_top_func", "jet_bot_func"};
        for (String name : funcNames) {
            List<UserFieldFunction> toDelete = new ArrayList<UserFieldFunction>();
            for (FieldFunction f : sim.getFieldFunctionManager().getObjects()) {
                if (!(f instanceof UserFieldFunction)) continue;
                UserFieldFunction uff = (UserFieldFunction) f;
                if (name.equals(uff.getFunctionName()) || name.equals(uff.getPresentationName())) {
                    toDelete.add(uff);
                }
            }
            if (!toDelete.isEmpty()) {
                for (UserFieldFunction uff : toDelete) {
                    log(sim, "Removing stale field function: " + uff.getPresentationName());
                }
                sim.getFieldFunctionManager().removeObjects(toDelete);
            }
        }
    }

    // ======================================================================= //
    // Helper: create or update a scalar UserFieldFunction [velocity dims]
    // ======================================================================= //

    private static UserFieldFunction getOrCreateScalarVelocityFunction(
            Simulation sim, String name, String definition) {

        // Delete every UserFieldFunction whose function name OR presentation name
        // matches — cleans up "jet_top_func 2" duplicates from previous runs.
        List<UserFieldFunction> toDelete = new ArrayList<UserFieldFunction>();
        for (FieldFunction f : sim.getFieldFunctionManager().getObjects()) {
            if (!(f instanceof UserFieldFunction)) continue;
            UserFieldFunction uff = (UserFieldFunction) f;
            if (name.equals(uff.getFunctionName()) || name.equals(uff.getPresentationName())) {
                toDelete.add(uff);
            }
        }
        if (!toDelete.isEmpty()) {
            sim.getFieldFunctionManager().removeObjects(toDelete);
        }

        // Always create fresh so name/type/dims are guaranteed correct.
        UserFieldFunction ff = sim.getFieldFunctionManager().createFieldFunction();
        ff.setPresentationName(name);
        ff.setFunctionName(name);
        ff.getTypeOption().setSelected(FieldFunctionTypeOption.Type.SCALAR);
        ff.setDimensions(Dimensions.Builder().velocity(1).build());
        ff.setDefinition(definition);
        return ff;
    }

    // ======================================================================= //
    // Helper: wire a field function to a boundary's VelocityMagnitudeProfile
    // ======================================================================= //

    private static void assignFunctionToBoundary(Boundary bnd, UserFieldFunction ff) {
        VelocityMagnitudeProfile prof = bnd.getValues().get(VelocityMagnitudeProfile.class);
        prof.setMethod(FunctionScalarProfileMethod.class);
        prof.getMethod(FunctionScalarProfileMethod.class).setFieldFunction(ff);
    }

    // ======================================================================= //
    // Shared utilities
    // ======================================================================= //

    private static void log(Simulation sim, String msg) {
        sim.println("[cylinder_afc_set_jets] " + msg);
    }

    private static String fmt(double v) {
        return String.format(java.util.Locale.US, "%.16g", v);
    }

    // ======================================================================= //
    // Minimal JSON parser (self-contained, no external dependencies)
    // ======================================================================= //

    static final class Json {
        static JsonObject parseFile(String path) {
            StringBuilder sb = new StringBuilder();
            try (BufferedReader br = new BufferedReader(new FileReader(path))) {
                String line;
                while ((line = br.readLine()) != null) sb.append(line).append('\n');
            } catch (IOException ex) {
                throw new RuntimeException("Could not read JSON config: " + path, ex);
            }
            Object parsed = new Parser(sb.toString()).parseValue();
            if (!(parsed instanceof Map)) {
                throw new RuntimeException("Top-level JSON must be an object.");
            }
            @SuppressWarnings("unchecked")
            JsonObject obj = new JsonObject((Map<String, Object>) parsed);
            return obj;
        }
    }

    static final class JsonObject {
        private final Map<String, Object> map;
        JsonObject(Map<String, Object> map) { this.map = map; }

        double getDouble(String path, double def) {
            Object v = get(path);
            if (v == null) return def;
            if (v instanceof Number) return ((Number) v).doubleValue();
            throw new RuntimeException("JSON path '" + path + "' is not numeric.");
        }

        int getInt(String path, int def) {
            Object v = get(path);
            if (v == null) return def;
            if (v instanceof Number) return ((Number) v).intValue();
            throw new RuntimeException("JSON path '" + path + "' is not integer-like.");
        }

        String getString(String path, String def) {
            Object v = get(path);
            if (v == null) return def;
            if (v instanceof String) return (String) v;
            throw new RuntimeException("JSON path '" + path + "' is not a string.");
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
        private final String s;
        private int i = 0;
        Parser(String s) { this.s = s; }

        Object parseValue() {
            skipWs();
            if (i >= s.length()) throw err("Unexpected end of input");
            char c = s.charAt(i);
            if (c == '{') return parseObject();
            if (c == '[') return parseArray();
            if (c == '"') return parseString();
            if (c == 't' || c == 'f') return parseBoolean();
            if (c == 'n') return parseNull();
            return parseNumber();
        }

        private Map<String, Object> parseObject() {
            expect('{');
            LinkedHashMap<String, Object> out = new LinkedHashMap<String, Object>();
            skipWs();
            if (peek('}')) { i++; return out; }
            while (true) {
                skipWs();
                String key = parseString();
                skipWs(); expect(':');
                Object value = parseValue();
                out.put(key, value);
                skipWs();
                if (peek('}')) { i++; break; }
                expect(',');
            }
            return out;
        }

        private List<Object> parseArray() {
            expect('[');
            ArrayList<Object> out = new ArrayList<Object>();
            skipWs();
            if (peek(']')) { i++; return out; }
            while (true) {
                out.add(parseValue());
                skipWs();
                if (peek(']')) { i++; break; }
                expect(',');
            }
            return out;
        }

        private String parseString() {
            expect('"');
            StringBuilder sb = new StringBuilder();
            while (i < s.length()) {
                char c = s.charAt(i++);
                if (c == '"') return sb.toString();
                if (c == '\\') {
                    if (i >= s.length()) throw err("Bad escape");
                    char e = s.charAt(i++);
                    switch (e) {
                        case '"':  sb.append('"');  break;
                        case '\\': sb.append('\\'); break;
                        case '/':  sb.append('/');  break;
                        case 'b':  sb.append('\b'); break;
                        case 'f':  sb.append('\f'); break;
                        case 'n':  sb.append('\n'); break;
                        case 'r':  sb.append('\r'); break;
                        case 't':  sb.append('\t'); break;
                        case 'u':
                            if (i + 4 > s.length()) throw err("Bad unicode escape");
                            sb.append((char) Integer.parseInt(s.substring(i, i + 4), 16));
                            i += 4; break;
                        default: throw err("Unsupported escape: \\" + e);
                    }
                } else {
                    sb.append(c);
                }
            }
            throw err("Unterminated string");
        }

        private Boolean parseBoolean() {
            if (s.startsWith("true",  i)) { i += 4; return Boolean.TRUE; }
            if (s.startsWith("false", i)) { i += 5; return Boolean.FALSE; }
            throw err("Invalid boolean");
        }

        private Object parseNull() {
            if (s.startsWith("null", i)) { i += 4; return null; }
            throw err("Invalid null");
        }

        private Number parseNumber() {
            int start = i;
            if (peek('-')) i++;
            while (i < s.length() && Character.isDigit(s.charAt(i))) i++;
            if (peek('.')) { i++; while (i < s.length() && Character.isDigit(s.charAt(i))) i++; }
            if (peek('e') || peek('E')) {
                i++;
                if (peek('+') || peek('-')) i++;
                while (i < s.length() && Character.isDigit(s.charAt(i))) i++;
            }
            String num = s.substring(start, i);
            try { return Double.valueOf(num); }
            catch (NumberFormatException ex) { throw err("Invalid number: " + num); }
        }

        private void skipWs() {
            while (i < s.length()) {
                char c = s.charAt(i);
                if (c == ' ' || c == '\n' || c == '\r' || c == '\t') i++;
                else break;
            }
        }

        private void expect(char c) {
            skipWs();
            if (i >= s.length() || s.charAt(i) != c) throw err("Expected '" + c + "'");
            i++;
        }

        private boolean peek(char c) { return i < s.length() && s.charAt(i) == c; }

        private RuntimeException err(String msg) {
            return new RuntimeException(msg + " at character " + i);
        }
    }
}