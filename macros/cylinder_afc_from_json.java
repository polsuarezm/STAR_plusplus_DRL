package macro;

import java.io.BufferedReader;
import java.io.FileReader;
import java.io.IOException;
import java.lang.reflect.Method;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
// NOTE: star.flow.UserFieldFunction and FunctionScalarProfileMethod imports
// are intentionally omitted here; they will be needed in the next macro (f(t,A,freq)).

import star.base.neo.*;
import star.cadmodeler.*;
import star.common.*;
import star.flow.*;
import star.material.*;
import star.meshing.*;
import star.prismmesher.*;
import star.vis.*;

public class cylinder_afc_from_json extends StarMacro {

    @Override
    public void execute() {
        Simulation sim = getActiveSimulation();

        String configPath = System.getenv("STARCCM_JSON");
        if (configPath == null || configPath.trim().isEmpty()) {
            configPath = "F:/STARCCM-UMICH/Nueva Carpeta/project-cylAFC/case_config.JSON";
        }

        log(sim, "Reading config from: " + configPath);
        JsonObject cfg = Json.parseFile(configPath);

        double D = cfg.getDouble("geometry.diameter", 1.0);
        double span = cfg.getDouble("geometry.span", 0.1);
        double xMin = cfg.getDouble("geometry.domain.x_min", -10.0 * D);
        double xMax = cfg.getDouble("geometry.domain.x_max", 15.0 * D);
        double yMin = cfg.getDouble("geometry.domain.y_min", -4.0 * D);
        double yMax = cfg.getDouble("geometry.domain.y_max",  4.0 * D);

        double rho = cfg.getDouble("flow.density", 1.0);
        double uIn = cfg.getDouble("flow.inlet_velocity", 1.0);
        double Re = cfg.getDouble("flow.reynolds", 100.0);
        double mu = rho * uIn * D / Re;

        double baseRef = cfg.getDouble("mesh.base_reference", D);
        double baseSize = cfg.getDouble("mesh.base_size_factor", 0.05) * baseRef;
        double targetSurface = cfg.getDouble("mesh.target_surface_factor", 0.025) * baseRef;
        double minSurface = cfg.getDouble("mesh.min_surface_factor", 0.01) * baseRef;
        double cylinderTarget = cfg.getDouble("mesh.cylinder_target_factor", 0.025) * baseRef;
        double cylinderMin = cfg.getDouble("mesh.cylinder_min_factor", 0.0125) * baseRef;
        double jetTarget = cfg.getDouble("mesh.jet_target_factor", 0.0125) * baseRef;
        double jetMin = cfg.getDouble("mesh.jet_min_factor", 0.005) * baseRef;
        double wakeCell = cfg.getDouble("mesh.wake_cell_factor", 0.02) * baseRef;
        int prismLayers = cfg.getInt("mesh.prism_layers", 8);
        double prismStretch = cfg.getDouble("mesh.prism_stretching", 1.2);
        double prismTotalThickness = cfg.getDouble("mesh.prism_total_thickness_factor", 0.05) * baseRef;

        // Jets: amplitude and frequency are read here for reference only.
        // In this vanilla macro the jets are set to v = 0 (quiescent).
        // The next macro will use these values to assign f(t, A, freq).
        // double amp  = cfg.getDouble("jets.amplitude",  0.1);
        // double freq = cfg.getDouble("jets.frequency", 0.16);

        double dt = cfg.getDouble("time.dt", 0.01);
        int steps = cfg.getInt("time.steps", 5000);
        String savePath = cfg.getString("output.save_sim", "output/cylinder_afc_out.sim");

        Units m = unit(sim, "m");
        Units nondim = unit(sim, "");
        Units kgm3 = unit(sim, "kg/m^3");
        Units pas = unit(sim, "Pa-s");

        // 1) Geometry / design params
        setDesignParameter(sim, "3D-CAD Model 1", "x0", Math.abs(xMin), m);
        setDesignParameter(sim, "3D-CAD Model 1", "x1", Math.abs(xMax), m);
        setDesignParameter(sim, "3D-CAD Model 1", "ytop", Math.abs(yMax), m);
        setDesignParameter(sim, "3D-CAD Model 1", "ybot", Math.abs(yMin), m);

        setExtrudeDistance(sim, "3D-CAD Model 1", "Extrude 1", span, m);

        setGlobalScalarParameter(sim, "Lx_ref", Math.max(1.0e-12, xMax), nondim);
        setBlockDimensions(
            sim,
            "3D-CAD Model 2",
            "Block 1",
            Math.max(1.0e-12, xMax),
            Math.max(1.0e-12, yMax - yMin),
            Math.max(1.0e-12, span),
            new double[] {0.5 * xMax, 0.5 * (yMax + yMin), 0.0},
            m
        );

        // 2) Mesh
        AutoMeshOperation meshOp =
            (AutoMeshOperation) sim.get(MeshOperationManager.class).getObject("Automated Mesh");
        AutoMeshDefaultValuesManager defaults = meshOp.getDefaultValues();

        defaults.get(BaseSize.class).setValueAndUnits(baseSize, m);
        defaults.get(PartsTargetSurfaceSize.class).getRelativeSizeScalar()
            .setValueAndUnits(targetSurface / baseSize, nondim);
        defaults.get(PartsMinimumSurfaceSize.class).getRelativeSizeScalar()
            .setValueAndUnits(minSurface / baseSize, nondim);

        PrismDefaultValuesManager prismDefaults = defaults.get(PrismDefaultValuesManager.class);
        prismDefaults.get(NumPrismLayers.class).getNumLayersValue().getQuantity().setValue(prismLayers);

        try {
            Object prismStretching = prismDefaults.get(PrismLayerStretching.class);
            setPrismStretchingValue(prismStretching, prismStretch, nondim);
        } catch (Exception ex) {
            log(sim, "Skipping prism stretching update; keeping template value. " + ex.getMessage());
        }

        PrismThickness globalPrism = prismDefaults.get(PrismThickness.class);
        globalPrism.getRelativeOrAbsoluteOption().setSelected(RelativeOrAbsoluteOption.Type.RELATIVE);
        globalPrism.getRelativeSizeScalar().setValueAndUnits(prismTotalThickness / baseSize, nondim);

        SurfaceCustomMeshControl cylCtrl = getSurfaceControl(meshOp, "surf-control-cylinder");
        if (cylCtrl != null) {
            cylCtrl.getCustomValues().get(PartsTargetSurfaceSize.class).getRelativeSizeScalar()
                .setValueAndUnits(cylinderTarget / baseSize, nondim);
            cylCtrl.getCustomValues().get(PartsMinimumSurfaceSize.class).getRelativeSizeScalar()
                .setValueAndUnits(cylinderMin / baseSize, nondim);

            try {
                PartsCustomizePrismMesh prismCond =
                    cylCtrl.getCustomConditions().get(PartsCustomizePrismMesh.class);
                prismCond.getCustomPrismOptions().setSelected(PartsCustomPrismsOption.Type.CUSTOMIZE);

                PartsCustomizePrismMeshControls prismCtrl = prismCond.getCustomPrismControls();
                prismCtrl.setCustomizeNumLayers(true);
                prismCtrl.setCustomizeTotalThickness(true);

                cylCtrl.getCustomValues().get(CustomPrismValuesManager.class).get(NumPrismLayers.class)
                    .getNumLayersValue().getQuantity().setValue(prismLayers);

                PrismThickness pt =
                    cylCtrl.getCustomValues().get(CustomPrismValuesManager.class).get(PrismThickness.class);
                pt.getRelativeOrAbsoluteOption().setSelected(RelativeOrAbsoluteOption.Type.RELATIVE);
                pt.getRelativeSizeScalar().setValueAndUnits(prismTotalThickness / baseSize, nondim);
            } catch (Exception ex) {
                log(sim, "Cylinder prism customization skipped: " + ex.getMessage());
            }
        }

        SurfaceCustomMeshControl jetCtrl = getSurfaceControl(meshOp, "surf-control-jets");
        if (jetCtrl != null) {
            jetCtrl.getCustomValues().get(PartsTargetSurfaceSize.class).getRelativeSizeScalar()
                .setValueAndUnits(jetTarget / baseSize, nondim);
            jetCtrl.getCustomValues().get(PartsMinimumSurfaceSize.class).getRelativeSizeScalar()
                .setValueAndUnits(jetMin / baseSize, nondim);
        }

        VolumeCustomMeshControl wakeCtrl = getVolumeControl(meshOp, "Volumetric Control");
        if (wakeCtrl != null) {
            VolumeControlSize vcs = wakeCtrl.getCustomValues().get(VolumeControlSize.class);
            vcs.getRelativeOrAbsoluteOption().setSelected(RelativeOrAbsoluteOption.Type.RELATIVE);
            vcs.getRelativeSizeScalar().setValueAndUnits(wakeCell / baseSize, nondim);
        }

        // 3) Flow properties from Re
        PhysicsContinuum physics =
            (PhysicsContinuum) sim.getContinuumManager().getContinuum("Physics 1");
        SingleComponentGasModel gasModel =
            physics.getModelManager().getModel(SingleComponentGasModel.class);
        Gas gas = (Gas) gasModel.getMaterial();

        ConstantMaterialPropertyMethod rhoMethod =
            (ConstantMaterialPropertyMethod) gas.getMaterialProperties()
                .getMaterialProperty(ConstantDensityProperty.class).getMethod();
        rhoMethod.getQuantity().setValueAndUnits(rho, kgm3);

        ConstantMaterialPropertyMethod muMethod =
            (ConstantMaterialPropertyMethod) gas.getMaterialProperties()
                .getMaterialProperty(DynamicViscosityProperty.class).getMethod();
        muMethod.getQuantity().setValueAndUnits(mu, pas);

        // 4) Boundaries
        Region fluid = sim.getRegionManager().getRegion("fluid");

        Boundary inlet = fluid.getBoundaryManager().getBoundary("inlet");
        setVelocityMagnitude(inlet, uIn);

        // Jets set to v = 0 (vanilla / quiescent initial condition).
        // Clean up any jet UserFieldFunctions left over from a previous run of
        // the set_jets macro so the sim starts from a known blank state.
        cleanJetFieldFunctions(sim);

        Boundary topJet = fluid.getBoundaryManager().getBoundary("jet_top");
        Boundary botJet = fluid.getBoundaryManager().getBoundary("jet_bot");

        setVelocityMagnitude(topJet, 0.0);
        setVelocityMagnitude(botJet, 0.0);

        // 5) Time setup
        setUnsteadyTimestep(sim, dt);
        setMaxSteps(sim, steps);

        // 6) Mesh, init, run, save
        log(sim, String.format(java.util.Locale.US,
            "Running case: Re=%.6g, D=%.6g, U=%.6g, mu=%.6g, baseRef=%.6g, dt=%.6g, steps=%d (jets v=0)",
            Re, D, uIn, mu, baseRef, dt, steps));

        meshOp.execute();

        Solution sol = sim.getSolution();
        sol.clearSolution();
        sol.initializeSolution();

        sim.getSimulationIterator().run();
        sim.saveState(savePath);
        log(sim, "Saved: " + savePath);
    }

    private static Units unit(Simulation sim, String name) {
        return (Units) sim.getUnitsManager().getObject(name);
    }

    private static void log(Simulation sim, String msg) {
        sim.println("[cylinder_afc_from_json] " + msg);
    }

    private static String fmt(double v) {
        return String.format(java.util.Locale.US, "%.16g", v);
    }

    private static CadModel getCadModel(Simulation sim, String cadModelName) {
        return (CadModel) sim.get(SolidModelManager.class).getObject(cadModelName);
    }

    private static void updateCadModel(CadModel cad) {
        tryInvokeNoArg(cad.getFeatureManager(), "updateModel");
    }

    private static void setVelocityMagnitude(Boundary bnd, double value) {
        VelocityMagnitudeProfile prof = bnd.getValues().get(VelocityMagnitudeProfile.class);
        prof.getMethod(ConstantScalarProfileMethod.class).getQuantity().setValue(value);
    }

    private static void cleanJetFieldFunctions(Simulation sim) {
        // Step 1: revert both jet boundaries to ConstantScalarProfileMethod (v=0)
        // so no field function is referenced before we try to delete them.
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
                    sim.println("[cylinder_afc_from_json] Could not reset boundary " + bName + ": " + ex.getMessage());
                }
            }
        } catch (Exception ex) {
            sim.println("[cylinder_afc_from_json] Could not access fluid region for boundary reset: " + ex.getMessage());
        }

        // Step 2: now safe to delete the orphaned field functions
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
                    sim.println("[cylinder_afc_from_json] Removing stale field function: " + uff.getPresentationName());
                }
                sim.getFieldFunctionManager().removeObjects(toDelete);
            }
        }
    }

    private static SurfaceCustomMeshControl getSurfaceControl(AutoMeshOperation op, String name) {
        try {
            return (SurfaceCustomMeshControl) op.getCustomMeshControls().getObject(name);
        } catch (Exception ex) {
            return null;
        }
    }

    private static VolumeCustomMeshControl getVolumeControl(AutoMeshOperation op, String name) {
        try {
            return (VolumeCustomMeshControl) op.getCustomMeshControls().getObject(name);
        } catch (Exception ex) {
            return null;
        }
    }

    private static void setDesignParameter(
        Simulation sim, String cadModelName, String paramName, double value, Units units
    ) {
        try {
            CadModel cad = getCadModel(sim, cadModelName);
            ScalarQuantityDesignParameter p =
                (ScalarQuantityDesignParameter) cad.getDesignParameterManager().getObject(paramName);
            p.getQuantity().setValueAndUnits(value, units);
            updateCadModel(cad);
        } catch (Exception ex) {
            throw new RuntimeException(
                "Could not set design parameter '" + paramName +
                "' in CAD model '" + cadModelName + "'.", ex);
        }
    }

    private static void setGlobalScalarParameter(Simulation sim, String name, double value, Units units) {
        try {
            ScalarGlobalParameter p =
                (ScalarGlobalParameter) sim.get(GlobalParameterManager.class).getObject(name);
            p.getQuantity().setValueAndUnits(value, units);
        } catch (Exception ex) {
            throw new RuntimeException("Could not set global parameter '" + name + "'.", ex);
        }
    }

    private static void setExtrudeDistance(
        Simulation sim, String cadModelName, String extrudeName, double span, Units units
    ) {
        try {
            CadModel cad = getCadModel(sim, cadModelName);
            ExtrusionMerge ext = (ExtrusionMerge) cad.getFeature(extrudeName);
            ext.getDistance().setValueAndUnits(span, units);
            updateCadModel(cad);
        } catch (Exception ex) {
            // Non-fatal
        }
    }

    private static void setBlockDimensions(
        Simulation sim,
        String cadModelName,
        String blockName,
        double length,
        double width,
        double height,
        double[] center,
        Units units
    ) {
        try {
            CadModel cad = getCadModel(sim, cadModelName);
            BlockPrimitiveFeature block = (BlockPrimitiveFeature) cad.getFeature(blockName);

            block.getLength().setValueAndUnits(length, units);
            block.getWidth().setValueAndUnits(width, units);
            block.getHeight().setValueAndUnits(height, units);

            block.getCenter().setCoordinateSystem(sim.getCoordinateSystemManager().getLabCoordinateSystem());
            block.getCenter().setUnits0(units);
            block.getCenter().setUnits1(units);
            block.getCenter().setUnits2(units);
            block.getCenter().setValue(new DoubleVector(center));

            updateCadModel(cad);
        } catch (Exception ex) {
            throw new RuntimeException("Could not set wake block dimensions.", ex);
        }
    }

    // NOTE: getOrCreateScalarVelocityFunction() is removed from this vanilla macro.
    // It will be reinstated in the next macro that wires up f(t, A, freq) field functions.
    // fmt() is also unused here but retained below for reference.
    private static void setUnsteadyTimestep(Simulation sim, double dt) {
        try {
            ImplicitUnsteadySolver solver = sim.getSolverManager().getSolver(ImplicitUnsteadySolver.class);
            solver.getTimeStep().setValue(dt);
        } catch (Exception ex) {
            log(sim, "Could not set timestep automatically: " + ex.getMessage());
        }
    }

    private static void setMaxSteps(Simulation sim, int steps) {
        try {
            StepStoppingCriterion crit =
                (StepStoppingCriterion) sim.getSolverStoppingCriterionManager()
                    .getSolverStoppingCriterion("Maximum Steps");
            crit.setMaximumNumberSteps(steps);
        } catch (Exception ex) {
            log(sim, "Could not set Maximum Steps automatically: " + ex.getMessage());
        }
    }

    private static void setPrismStretchingValue(Object prismStretching, double value, Units units) {
        if (prismStretching == null) return;

        if (tryNestedQuantitySetter(prismStretching, "getStretchingFactor", value, units)) return;
        if (tryNestedQuantitySetter(prismStretching, "getRelativeSizeScalar", value, units)) return;
        if (trySetter(prismStretching, "setValueAndUnits", value, units)) return;
        if (trySetter(prismStretching, "setValue", value)) return;

        return;
    }

    private static boolean tryNestedQuantitySetter(Object owner, String getter, double value, Units units) {
        try {
            Method gm = owner.getClass().getMethod(getter);
            Object q = gm.invoke(owner);
            if (q == null) return false;

            try {
                Method m = q.getClass().getMethod("setValueAndUnits", double.class, Units.class);
                m.invoke(q, value, units);
                return true;
            } catch (NoSuchMethodException ignored) { }

            try {
                Method m = q.getClass().getMethod("setValue", double.class);
                m.invoke(q, value);
                return true;
            } catch (NoSuchMethodException ignored) { }

            return false;
        } catch (Exception ex) {
            return false;
        }
    }

    private static boolean trySetter(Object owner, String methodName, Object... args) {
        try {
            Class<?>[] types = new Class<?>[args.length];
            for (int i = 0; i < args.length; i++) {
                if (args[i] instanceof Double) types[i] = double.class;
                else if (args[i] instanceof Integer) types[i] = int.class;
                else types[i] = args[i].getClass();
            }
            Method m = owner.getClass().getMethod(methodName, types);
            m.invoke(owner, args);
            return true;
        } catch (Exception ex) {
            return false;
        }
    }

    private static void tryInvokeNoArg(Object owner, String methodName) {
        try {
            Method m = owner.getClass().getMethod(methodName);
            m.invoke(owner);
        } catch (Exception ex) {
            // silently ignore
        }
    }

    // ---------- Tiny JSON parser ----------

    static final class Json {
        static JsonObject parseFile(String path) {
            StringBuilder sb = new StringBuilder();
            try (BufferedReader br = new BufferedReader(new FileReader(path))) {
                String line;
                while ((line = br.readLine()) != null) {
                    sb.append(line).append('\n');
                }
            } catch (IOException ex) {
                throw new RuntimeException("Could not read JSON config: " + path, ex);
            }
            Object parsed = new Parser(sb.toString()).parseValue();
            if (!(parsed instanceof Map)) {
                throw new RuntimeException("Top-level JSON must be an object.");
            }
            return new JsonObject((Map<String, Object>) parsed);
        }
    }

    static final class JsonObject {
        private final Map<String, Object> map;
        JsonObject(Map<String, Object> map) { this.map = map; }

        double getDouble(String path, double defaultValue) {
            Object v = get(path);
            if (v == null) return defaultValue;
            if (v instanceof Number) return ((Number) v).doubleValue();
            throw new RuntimeException("JSON path '" + path + "' is not numeric.");
        }

        int getInt(String path, int defaultValue) {
            Object v = get(path);
            if (v == null) return defaultValue;
            if (v instanceof Number) return ((Number) v).intValue();
            throw new RuntimeException("JSON path '" + path + "' is not integer-like.");
        }

        String getString(String path, String defaultValue) {
            Object v = get(path);
            if (v == null) return defaultValue;
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
                skipWs();
                expect(':');
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
                    if (i >= s.length()) throw err("Bad escape sequence");
                    char e = s.charAt(i++);
                    switch (e) {
                        case '"': sb.append('"'); break;
                        case '\\': sb.append('\\'); break;
                        case '/': sb.append('/'); break;
                        case 'b': sb.append('\b'); break;
                        case 'f': sb.append('\f'); break;
                        case 'n': sb.append('\n'); break;
                        case 'r': sb.append('\r'); break;
                        case 't': sb.append('\t'); break;
                        case 'u':
                            if (i + 4 > s.length()) throw err("Bad unicode escape");
                            String hex = s.substring(i, i + 4);
                            sb.append((char) Integer.parseInt(hex, 16));
                            i += 4;
                            break;
                        default:
                            throw err("Unsupported escape: \\" + e);
                    }
                } else {
                    sb.append(c);
                }
            }
            throw err("Unterminated string");
        }

        private Boolean parseBoolean() {
            if (s.startsWith("true", i)) { i += 4; return Boolean.TRUE; }
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
            if (peek('.')) {
                i++;
                while (i < s.length() && Character.isDigit(s.charAt(i))) i++;
            }
            if (peek('e') || peek('E')) {
                i++;
                if (peek('+') || peek('-')) i++;
                while (i < s.length() && Character.isDigit(s.charAt(i))) i++;
            }
            String num = s.substring(start, i);
            try {
                return Double.valueOf(num);
            } catch (NumberFormatException ex) {
                throw err("Invalid number: " + num);
            }
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

        private boolean peek(char c) {
            return i < s.length() && s.charAt(i) == c;
        }

        private RuntimeException err(String msg) {
            return new RuntimeException(msg + " at character " + i);
        }
    }
}