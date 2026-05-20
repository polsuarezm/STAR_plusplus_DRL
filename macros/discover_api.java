package macro;

import java.io.*;
import java.lang.reflect.*;
import java.net.*;
import java.util.*;
import java.util.jar.*;

import star.common.*;

/**
 * discover_api.java
 *
 * Diagnostic macro — run once to find:
 *   1. The correct fully-qualified class names for PointProbe and ReportMonitor
 *   2. The correct field function names (e.g. Velocity Magnitude)
 *   3. The correct class name for a Point-type derived part (for probe sampling)
 *   4. The PartManager API for creating derived parts
 *
 * Output is printed to the STAR-CCM+ output window.
 * Delete this macro after use.
 */
public class discover_api extends StarMacro {

    @Override
    public void execute() {
        Simulation sim = getActiveSimulation();

        /* ── 1. Field functions ───────────────────────────────────────── */
        sim.println("\n=== FIELD FUNCTIONS ===");
        try {
            for (FieldFunction ff : sim.getFieldFunctionManager().getObjects()) {
                sim.println(String.format("  %-45s  fn='%s'",
                    ff.getPresentationName(), ff.getFunctionName()));
            }
        } catch (Exception ex) {
            sim.println("ERROR listing field functions: " + ex.getMessage());
        }

        /* ── 2. Report types via class-name scan ─────────────────────── */
        sim.println("\n=== STAR-CCM+ CLASSES containing 'Probe' or 'Monitor' or 'Report' ===");
        sim.println("(scanning classloader jars — may take a few seconds)");
        Set<String> allHits = new TreeSet<String>();
        try {
            ClassLoader cl = sim.getClass().getClassLoader();
            scanClassLoader(cl, allHits, new String[]{"Probe", "Monitor", "Report"});
            if (allHits.isEmpty()) {
                sim.println("  (no jar-based classloader found — trying forName scan)");
                forNameScan(sim);
            } else {
                for (String name : allHits)
                    sim.println("  " + name);
            }
        } catch (Exception ex) {
            sim.println("Scan error: " + ex.getMessage());
            forNameScan(sim);
        }

        /* ── 3. Point / Part class scan ──────────────────────────────── */
        sim.println("\n=== STAR-CCM+ CLASSES containing 'Point' AND 'Part' (derived parts) ===");
        Set<String> partHits = new TreeSet<String>();
        try {
            ClassLoader cl = sim.getClass().getClassLoader();
            scanClassLoaderBoth(cl, partHits, "Point", "Part");
            if (partHits.isEmpty()) {
                sim.println("  (none found via jar scan)");
            } else {
                for (String name : partHits)
                    sim.println("  " + name);
            }
        } catch (Exception ex) {
            sim.println("Part scan error: " + ex.getMessage());
        }

        /* ── 4. Candidate Point part classes via forName ─────────────── */
        sim.println("\n=== CANDIDATE POINT PART classes (forName) ===");
        String[] partCandidates = {
            "star.vis.PointPart",
            "star.derived.PointPart",
            "star.base.vis.PointPart",
            "star.common.PointPart",
            "star.post.PointPart",
            "star.vis.ProbePoint",
            "star.vis.PointAnnotation",
            "star.common.ProbePoint",
            "star.post.ProbePoint",
            "star.derived.ProbePoint",
        };
        for (String name : partCandidates) {
            try {
                Class.forName(name);
                sim.println("  FOUND: " + name);
            } catch (ClassNotFoundException ex) {
                sim.println("  not found: " + name);
            }
        }

        /* ── 5. PartManager API ───────────────────────────────────────── */
        sim.println("\n=== PART MANAGER methods ===");
        try {
            Object pm = sim.getPartManager();
            sim.println("  PartManager class: " + pm.getClass().getName());
            for (Method m : pm.getClass().getMethods()) {
                String n = m.getName();
                if (n.contains("create") || n.contains("Create") || n.contains("Part") || n.contains("Point")) {
                    sim.println("  " + m.toGenericString());
                }
            }
        } catch (Exception ex) {
            sim.println("  PartManager error: " + ex.getMessage());
        }

        /* ── 6. DerivedPartManager (if separate) ─────────────────────── */
        sim.println("\n=== DERIVED PART MANAGER (via reflection on sim) ===");
        for (Method m : sim.getClass().getMethods()) {
            String n = m.getName();
            if ((n.contains("Derived") || n.contains("Part")) && m.getParameterCount() == 0) {
                try {
                    Object result = m.invoke(sim);
                    if (result != null) {
                        sim.println("  sim." + n + "() = " + result.getClass().getName());
                        // list create methods on this manager
                        for (Method mm : result.getClass().getMethods()) {
                            String mn = mm.getName();
                            if (mn.contains("create") || mn.contains("Create") || mn.contains("Point")) {
                                sim.println("    -> " + mm.toGenericString());
                            }
                        }
                    }
                } catch (Exception ignored) {}
            }
        }

        /* ── 7. Available report types via ReportManager ─────────────── */
        sim.println("\n=== REPORT MANAGER available types ===");
        try {
            Method m = sim.getReportManager().getClass().getMethod("getAvailableTypes");
            Object result = m.invoke(sim.getReportManager());
            sim.println("  " + result);
        } catch (Exception ignored) {}
        try {
            Method m = sim.getReportManager().getClass().getMethod("getAvailableTypeList");
            Object result = m.invoke(sim.getReportManager());
            sim.println("  " + result);
        } catch (Exception ignored) {}
        for (Method m : sim.getReportManager().getClass().getMethods()) {
            if (m.getName().contains("Type") && m.getParameterCount() == 0) {
                try {
                    Object r = m.invoke(sim.getReportManager());
                    sim.println("  " + m.getName() + "() = " + r);
                } catch (Exception ignored) {}
            }
        }

        sim.println("\n=== DONE ===");
    }

    /** Scan classloader jars for classes whose simple name contains ANY of the given tokens. */
    private static void scanClassLoader(ClassLoader cl, Set<String> hits, String[] tokens) throws Exception {
        if (cl == null) return;
        scanClassLoader(cl.getParent(), hits, tokens);
        if (!(cl instanceof URLClassLoader)) return;
        URL[] urls = ((URLClassLoader) cl).getURLs();
        for (URL url : urls) {
            String path = url.getFile();
            if (!path.endsWith(".jar")) continue;
            try (JarFile jar = new JarFile(path)) {
                Enumeration<JarEntry> entries = jar.entries();
                while (entries.hasMoreElements()) {
                    String entry = entries.nextElement().getName();
                    if (!entry.endsWith(".class")) continue;
                    String className = entry.replace('/', '.').replace(".class", "");
                    if (!className.startsWith("star.")) continue;
                    String simple = className.substring(className.lastIndexOf('.') + 1);
                    for (String token : tokens) {
                        if (simple.contains(token)) { hits.add(className); break; }
                    }
                }
            } catch (Exception ignored) {}
        }
    }

    /** Scan for classes whose simple name contains BOTH token1 AND token2. */
    private static void scanClassLoaderBoth(ClassLoader cl, Set<String> hits, String t1, String t2) throws Exception {
        if (cl == null) return;
        scanClassLoaderBoth(cl.getParent(), hits, t1, t2);
        if (!(cl instanceof URLClassLoader)) return;
        URL[] urls = ((URLClassLoader) cl).getURLs();
        for (URL url : urls) {
            String path = url.getFile();
            if (!path.endsWith(".jar")) continue;
            try (JarFile jar = new JarFile(path)) {
                Enumeration<JarEntry> entries = jar.entries();
                while (entries.hasMoreElements()) {
                    String entry = entries.nextElement().getName();
                    if (!entry.endsWith(".class")) continue;
                    String className = entry.replace('/', '.').replace(".class", "");
                    if (!className.startsWith("star.")) continue;
                    String simple = className.substring(className.lastIndexOf('.') + 1);
                    if (simple.contains(t1) && simple.contains(t2)) hits.add(className);
                }
            } catch (Exception ignored) {}
        }
    }

    /** Fallback: try known candidate names via Class.forName */
    private static void forNameScan(Simulation sim) {
        String[] candidates = {
            "star.common.PointProbe",
            "star.common.ReportMonitor",
            "star.vis.PointProbe",
            "star.base.neo.PointProbe",
            "star.base.report.PointProbe",
            "star.report.PointProbe",
            "star.common.monitor.ReportMonitor",
            "star.monitor.ReportMonitor",
            "star.base.report.ReportMonitor",
            "star.report.ReportMonitor",
        };
        sim.println("  Trying candidate class names:");
        for (String name : candidates) {
            try {
                Class.forName(name);
                sim.println("  FOUND: " + name);
            } catch (ClassNotFoundException ex) {
                sim.println("  not found: " + name);
            }
        }
    }
}
