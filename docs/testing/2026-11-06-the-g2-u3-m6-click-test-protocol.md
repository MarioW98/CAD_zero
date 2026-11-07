# The G2-u3 m6 click-test protocol — the phase's exit

**What this is.** The protocol of the maintainer's click-test, the act
that closes G2-u3 and with it the fillet line's phase: the suite
covers the contracts, the hands cover the experience (ADR-0030/0032's
lesson, the third phase running). The authority is ADR-0039 D6's rung
m6 — "the guide case end-to-end + the click-test: the chain (the box
→ the blind hole → the fillet on the rim) with the printed,
hand-checkable table extended by the D1.2 rows; the radius edit
bit-identical through ONE undo; the v0.5 round-trip; **the phase's
exit: the user's click-test**" (docs/adr/0039-the-fillet.md, the D6
morsi list, item 5 — lines 823-829 of the accepted document). Two
landed facts sharpen the rung's words, both already registered: the
round-trip's number is **v0.6.0** (the register's arbitration — the
v0.5 MINOR went to the sketch line, the fillet's m4 rebased on it:
the register's m4 row), and the printed hand-checkable tables are the
**Debug report's two name-column tables** (face/edge — the m5 row:
"the two NAME-COLUMN tables … the m2/m3 surfaces consumed at last"),
the D1.2 rows riding the rim case's own table.

**The form.** The G1 precedent (2026-10-26, the register's Phase G
row): *the D7.5 table read in the report, the volume verified by
hand, the radius edited live* — three acts, one session, the register
row carrying the verbalization. This protocol is the same form grown
to the fillet's chain: eight steps, every expected value a KERNEL
number cited from the register or the golden (never re-derived here),
the whole executable by hand on the workbench — with one declared
exception: the class-4 death's re-author (step 7), for which v1 has no
canvas gesture (the m5 test's own declaration: "v1 has no delete
gesture, the API route is the established test driver") — that one
prep act rides the scene's entity doors through the driver quoted
below, while the trigger edit and everything verified are the
maintainer's own hands on the live panel.

**The verdict.** The register's m6 row lands only after this
protocol has been executed and the maintainer's verdict has arrived.
A step that fails is a defect reported, never a silent fix: a defect
found by the click-test is a correction with its own bite.

---

## Step 1 — The launch

**The cold road, ONCE per clone.** From a fresh clone, the build the
CI judges:

```bash
cmake -B build -G Ninja \
    -DCMAKE_BUILD_TYPE=Release \
    -DCAD_0_BUILD_PYTHON=ON \
    -DCAD_0_BUILD_TESTING=ON \
    -DPython3_EXECUTABLE=$(which python3)
cmake --build build
```

**The warm road, every click-test after that — the m7 cure
(2026-11-08).** The maintainer's tax, by name and by his own words
(the complaint verbalized 2026-11-07): a FULL build, "~10+ min" on
his Windows box, paid at every click-test. The diagnosis, measured
on the tip in the sandbox (2 CPU, Linux-side — the sandbox has no
Windows, the numbers say what they measured): the pull is INNOCENT
(a fetch with nothing new: 0.64 s); the tax is the from-zero build
alone — **326.54 s** (configure 11.66 s + build 314.88 s, 127 ninja
steps) — while the incremental after a typical bite
(`core/brep/src/tessellate.cpp` and
`bindings/python/src/scene_bindings.cpp` touched — the -bis bites'
own files) is **13.64 s, 4.2% of the tax** (two translation units,
the relinks, the copy step). The cure, three moves — documented
here, no wrapper scripts:

1. **The build directory is never deleted between pulls** — the
   incremental build is the road, always (`cmake --build build`
   alone; a changed `CMakeLists.txt` re-configures by itself,
   ninja's own rule);
2. **ccache as the compiler launcher** (`CMAKE_C_COMPILER_LAUNCHER`
   and `CMAKE_CXX_COMPILER_LAUNCHER = ccache`): even a wiped build
   tree or a reconfigure finds the cache — measured, a full rebuild
   after wiping the build directory: **7.97 s, 2.4% of the tax**
   (reconfigure 0.85 s + build 7.12 s, 112/112 direct hits); on
   MSYS2/MINGW64: `pacman -S mingw-w64-x86_64-ccache`;
3. **The Ninja generator** (`cmake -G Ninja`): exact dependencies,
   no over-recompilation; on MSYS2/MINGW64:
   `pacman -S mingw-w64-x86_64-ninja`.

The commands, the maintainer's environment verbatim — MSYS2/MINGW64
for the shell, PowerShell for the desktop side:

```bash
# MSYS2 MINGW64 — ONCE (the tools):
pacman -S mingw-w64-x86_64-ccache mingw-w64-x86_64-ninja

# MSYS2 MINGW64 — ONCE per clone (the cold road, configured WITH the
# cure so every later build walks warm):
cmake -B build -G Ninja \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_C_COMPILER_LAUNCHER=ccache \
    -DCMAKE_CXX_COMPILER_LAUNCHER=ccache \
    -DCAD_0_BUILD_PYTHON=ON \
    -DCAD_0_BUILD_TESTING=ON \
    -DPython3_EXECUTABLE=$(which python3)
cmake --build build

# MSYS2 MINGW64 — an EXISTING build directory gains the cure with one
# reconfigure (measured: 0.85 s — the build dir is kept, never wiped):
cmake -B build -DCMAKE_C_COMPILER_LAUNCHER=ccache \
    -DCMAKE_CXX_COMPILER_LAUNCHER=ccache

# MSYS2 MINGW64 — EVERY click-test (the warm road):
git pull
cmake --build build
```

```powershell
# PowerShell, repo root — the workbench's own entry:
$env:PYTHONPATH = "build\python"
python -m CAD_0.ui
```

**ADR-0014, declared — the cache must not be able to change one bit
of output** ([ADR-0014](../adr/0014-floating-point-determinism.md)).
ccache is transparent, and this is verified, not assumed: at the
same build path with the same flags, the digests of every built
artifact (30 files across `bin/`, `lib/` and `python/`) are
**bit-identical** across the no-ccache build, the cache-cold build
and the cache-warm build, and this protocol's own execution
goldens — every expected value of the eight steps probed through
the scene/report doors, no GL — print byte-identical under each
(the suite green on the built tree: the C++ runner, the Python
suite under Xvfb). The limits, declared: the FIRST build always
pays its minutes — the cache makes the following ones fast (the
cache-cold full build costs the tax, 99.3% of it); the numbers
above are Linux-side, and the Windows verification is the
maintainer's own next pull, declared here as his; the structural
cure — a Windows CI column with downloadable artifacts, zero local
compilation — stays pinned where it is (P1, Phase 1; the registered
unit and its seen price:
[the CI runner audit](../audits/2026-11-01-ci-runner-asan-and-attestations.md),
B3): this bite does not touch it.

The workbench (the app's own entry, `python -m CAD_0.ui` — the
`__main__` docstring's command, run from the repo root):

```bash
PYTHONPATH=build/python python -m CAD_0.ui
```

Expected: the window **"CAD_0 — Phase C+ Desktop"**, the menus
**File / Create / Feature / Constraints / Edit / View / Debug /
Help**, the status bar reading **"Ready — try Create → Sphere"**.
(On a headless box the Codespaces route of the README —
`bash scripts/start_codespaces.sh` — serves the same build.)

## Step 2 — The guide box, the selection by name

1. **Create → Box**: the tree gains a `SDF` row; the panel shows
   `extent_x/extent_y/extent_z` = `0.5` (the registry's defaults).
2. Set the three extents to **1.0** in the panel's editors — the box
   [−1,1]³, the guide geometry (ADR-0039 D5, the box guide case).
3. With the Box row selected, **Feature → Fillet…**: the dialog
   "Fillet — select an edge, set the radius" opens; the operand combo
   pre-selects **`Box (id 1) — 12 edge(s) named`**.
4. The edge list carries **12 rows, all named** (the kernel's own
   query): `box#1:bottom0…3`, `box#1:top0…3`, `box#1:vertical0…3`;
   the unnamed declaration line stays **empty** (the box's ring is
   all named).
5. Click the row **`box#1:vertical2`**.

**Expected:** the reference row reads, character for character,
**`Reference: box#1:vertical2`** — the name the kernel uses, the
string the recipe will store (ADR-0039 D5 line 659 pins this edge;
the m5 row: "the reference shown CHARACTER FOR CHARACTER").

## Step 3 — The fillet at r = 0.5, the report saying everything

1. In the dialog, set **Radius = 0.5**, press **Apply**. The dialog
   closes; the tree gains the row **`Fillet_2`** of type **`FIL`**;
   the status bar reads **"Created Fillet_2"**; ONE undo entry
   exists for the act.
2. The panel (the FIL row is the selection) shows:
   - **Kind** = `Fillet (exact-side feature — the rolling ball)`
   - **radius** = `0.500000` (the authored parameter, editable)
   - **Edge reference** = `box#1:vertical2`
   - **Counts (V/E/F/H)** = `10/15/7/0`
   - **χ (V−E+F−H)** = `2`
   - **Volume** = `7.89269908 (exact)`
   - **Born face(s)** = `fil#2:flank[box#1:back × box#1:right] [cylinder]`
   - **Born edge names** = the two tangent wires
     `fil#2:isect[box#1:back × fil#2:flank[box#1:back × box#1:right]]`,
     `fil#2:isect[box#1:right × fil#2:flank[box#1:back × box#1:right]]`
3. **Debug → Body report — topology, mass, provenance** (the FIL row
   selected) prints:
   - `V = 10, E = 15, F = 7, H (inner loops) = 0`
   - `χ = V - E + F - H = 10 - 15 + 7 - 0 = 2`, `Genus = 0`
   - `Mass properties … exact (all_exact = True)`,
     `Volume = 7.89269908`
   - the **face table**: 7 rows — the six kept `box#1:*` faces plus
     `fil#2:flank[box#1:back × box#1:right] | cylinder`
   - the **edge table**: 15 rows — the kept names, the two tangent
     wires, and the two quarter arcs **honestly `-`** (the name
     column's own honesty).

**Sources:** ADR-0039 D5 lines 658-664 (the box guide case
pre-pinned: V=10, E=15, F=7, H=0, χ=2, the volume 7.5+π/8 ≈
7.892699081698725, all_exact); the register's m2 row (the golden,
the names, the census 30/2); the m2 C++ golden
(tests/unit/brep/test_g2_u3_m2_fillet.cpp); the flank's name pinned
in the m2 row. The hand-check: 7.5 + π/8 by hand beside the report.

## Step 4 — The out-of-band radius, the refusal in words with the bound

1. **Feature → Fillet…** again; in the operand combo choose
   **`Box (id 1)`**; click the row `box#1:vertical2`; set
   **Radius = 2.5**; press **Apply**.

**Expected:** the dialog **stays open** and its status area quotes
the kernel's refusal verbatim (the text is selectable):

> `fillet_edge: the tangent line exits the first adjacent face (fillet_radius_exceeds_face): the radius 2.500000 against the typed band's bound 2.000000 - 0.000001 (the face's own reach minus the declared Linear tolerance): the tangent line would exit the face, and the flank's closed forms would place it outside the material. Remedy: shrink the radius below 1.999999, or restructure the body (the band is the face's own reach, ADR-0039 D2's derived decision).`

The offending number **and** the bound are inside the words. Nothing
was written: no node, no undo entry — the tree still shows `Box` and
`Fillet_2`, the filleted body intact. Press **Cancel**.

**Sources:** the register's m5 row — "fillet_radius_exceeds_face
(2.500000 against 2.000000 − 0.000001)" — pinned by the m5 UI test;
the message's shape is the kernel's own (core/brep fillet.hpp's band
refusal, std::to_string's six decimals).

## Step 5 — The radius edit, class 1 silent, ONE undo

1. Select the `Fillet_2` row; in the panel set the **radius** editor
   to **0.25** (type it, commit with Return).

**Expected — silent:** no dialog, no re-selection; the status bar
speaks the re-run's answer: **"Set radius = 0.25 — the feature re-ran
on the new radius (the reference taken once)"**. The counts row stays
**`10/15/7/0`** (fixed for every in-band r — the m2 row's edit
contract), the **Edge reference** row unchanged, and the **Volume**
row is the closed form's own value at the new r: **`7.97317477
(exact)`** (abc − ℓ·r²(1−π/4) at r=0.25 = 7.875+π/32 ≈
7.973174770424681 — the D5 formula, the m5 row's "the volume row the
closed form's own value at the new r").

2. **Edit → Undo** once: the radius returns to 0.5, the Volume row to
   `7.89269908 (exact)`. **Edit → Redo**: 0.25 and `7.97317477`
   again — bit-identical numbers, ONE entry each way.

## Step 6 — The blind rim, the m3 guide case, the pinned string

1. **File → New Scene** (Discard, if asked).
2. **Create → Box**, the three extents to **1.0** (node id 1).
3. **Create → Sketch** (id 2): the canvas tab opens. Pick the
   **Circle** tool (shortcut **3**), press at the centre near the
   origin, release on the rim. Pick **Select** (**1**), click the
   circle: the panel shows its parameters — set **cx = 0**, **cy =
   0**, **r = 0.5** exactly (the `hole` checkbox stays unchecked).
4. Back on the 3D tab, click the `Sketch` row; **Feature →
   Extrude…**: distance **2.0**, direction **+Z**, OK — the `FEAT`
   row (id 3).
5. Ctrl-click the `Box` and `Extrude` rows; **Edit → Subtract…**: the
   dialog "Subtract (boolean difference)" with target **Box (id 1)**,
   tool **Extrude (id 3)** (the defaults), offsets **0 / 0 / −0.5**,
   OK — the `CSG` row (id 4); the operands hide.
6. Select the CSG row; **Debug → Body report**: the blind body's own
   numbers — `V = 12, E = 17, F = 8, H (inner loops) = 1`, `χ = 2`,
   `Volume = 6.82190275` (exact; 8 − 0.375π — the u2 golden, the
   additivity unmoved).
7. With the CSG row selected, **Feature → Fillet…**: the operand
   combo pre-selects the Subtract; the edge list carries **15 named
   rows** (the box's 12, the two `isect` rims, the seam — the
   seam-wired pair taken whole, one row per logical name). Click:

   **`sub#4:isect[box#1:top × ext#3:lat(0)]@hi`**

   — the pinned string (ADR-0038 D3, quoted at ADR-0039 D5 line 667),
   landing character for character because the hand order IS the
   golden's order (box#1, ext#3, sub#4 — the DAG ids the morse-5
   rule made the node ids). The reference row shows it verbatim.
8. Set **Radius = 0.25**, **Apply**.

**Expected:** the row **`Fillet_5`** (`FIL`); the panel:
- **Counts (V/E/F/H)** = `14/20/9/1`, **χ (V−E+F−H)** = `2`
- **Volume** = `6.77505976 (exact)`
- **Born face(s)** = `fil#5:flank[box#1:top × ext#3:lat(0)] [torus]`
- **Born edge names** = the two tangent wires and the flank's own
  seam: `fil#5:isect[box#1:top × fil#5:flank[box#1:top ×
  ext#3:lat(0)]]`, `fil#5:isect[ext#3:lat(0) × fil#5:flank[box#1:top
  × ext#3:lat(0)]]`, `fil#5:seam[fil#5:flank[box#1:top ×
  ext#3:lat(0)]]`
- **Edge reference** = the rim string, verbatim.

**Debug → Body report** on the FIL row: `V = 14, E = 20, F = 9,
H (inner loops) = 1`, `χ = 2`, `Genus = 0`, `Volume = 6.77505976`,
the centroid on the axis, and the **name-column tables** — the D1.2
rows in the flesh: the flank's `torus` family beside its name, the
seam, the kept `box#1:*`/`ext#3:lat(0)` names, the honestly `-` rows
beside them. The hand-check: (8 − 0.375π) − V_r with V_r = 2π[a·r²(1−π/4)
+ r³(5/6−π/4)] at a=0.5, r=0.25 — the wedge and the toroidal mass
family agreeing (the m3 row's two-arithmetic pin).

**Sources:** ADR-0039 D5 lines 666-675 (the rim guide case
pre-pinned: the reference string, 14/20/9/H=1/χ=2, the volume ≈
6.775059758517764 analytic — the kernel's own double landing at
6.7750597585177665, the register's m4 corpus row); the m3 register
row (the names, the census 9/22/16, the volume, the tavola 47 rows);
the m3 C++ golden (tests/unit/brep/test_g2_u3_m3_toroidal.cpp); the
m4 corpus golden (tests/golden/native_v0_6_0.CAD_0).

**Then save** (this is step 8's first half, done now so the file
captures the good state): **File → Save As…** → for instance
`m6_blind_fillet.CAD_0`. The atomic save lands; the title bar's `*`
clears. The file declares `"version": "0.6.0"` (the v0.6 rebased
bump, the register's m4 row).

## Step 7 — The class-4 death, the payload shown, the body kept

This step runs in its own session, through the driver below: v1 has
no canvas gesture that replaces an entity (the m5 test's own
declaration), so the re-author rides the scene's entity doors — the
same doors the canvas's own write family uses — while the placement
edit that fires the ONE rebuild, and everything verified, are the
maintainer's hands on the live window.

Save this driver as `m6_death_session.py` (repo root) and run it:

```bash
PYTHONPATH=build/python python3 m6_death_session.py
```

```python
#!/usr/bin/env python3
"""The m6 click-test's death session (step 7): the real window, the
guide chain and the re-author through the scene's own doors, the
trigger edit and the verification in the maintainer's hands."""
from PySide6.QtWidgets import QApplication


def main() -> None:
    app = QApplication.instance() or QApplication([])
    from CAD_0 import brep as brep_mod
    from CAD_0 import math as math_mod
    from CAD_0 import scene as scene_mod
    from CAD_0 import sdf as sdf_mod
    from CAD_0.ui.main_window import MainWindow

    win = MainWindow()
    win._current_path = None

    # The guide chain through the scene doors (the m5 test's own
    # recipe, the m3 id order — the pinned strings land):
    # box#1, sketch#2, extrude#3, subtract#4, fillet#5.
    bx = win._scene.add_node("Box", sdf_mod.box(
        math_mod.Vec3f(1.0, 1.0, 1.0)))
    node = win._scene.get_node(bx)
    node.builder = "box"
    node.set_params({"extent_x": 1.0, "extent_y": 1.0,
                     "extent_z": 1.0})
    chains = [scene_mod.SketchChain(
        [scene_mod.sketch_circle((0.0, 0.0), 0.5, False)])]
    sk = win._scene.add_sketch_node("Sketch", chains, [])
    ex = win._scene.add_extrude_node(
        "Extrude", sketch=sk, distance=2.0, direction=1)
    sub = win._scene.add_operation_node(
        "Subtract", target=bx, tool=ex,
        offset_x=0.0, offset_y=0.0, offset_z=-0.5)
    win._rebuild_ui_from_scene()

    # The rim, taken from the operand's own query (never typed
    # by hand — the grammar's own derivation over the DAG ids).
    blind = win._scene.get_node(sub)
    rim = None
    for i in range(brep_mod.edge_count(blind.exact_brep)):
        e = brep_mod.edge_query(blind.exact_brep, i)
        if "@hi" in e["name"] and "isect" in e["name"]:
            rim = e["name"]
            break
    print("the rim (the reference, verbatim):", rim)

    # The fillet through the window's own Apply route (the same
    # command-wrapped route the dialog's Apply button fires).
    fid = win._apply_fillet(rim, 0.25)
    print("the fillet node id:", fid)

    # THE RE-AUTHOR (the quartet's own scenario): the tool's
    # circle REPLACED by the square chain — the scene's entity doors.
    win._scene.sketch_set_entity(sk, 0, 0, scene_mod.sketch_segment(
        (-0.5, -0.5), (0.5, -0.5), True))
    win._scene.sketch_add_entity(sk, 0, scene_mod.sketch_segment(
        (0.5, -0.5), (0.5, 0.5), True))
    win._scene.sketch_add_entity(sk, 0, scene_mod.sketch_segment(
        (0.5, 0.5), (-0.5, 0.5), True))
    win._scene.sketch_add_entity(sk, 0, scene_mod.sketch_segment(
        (-0.5, 0.5), (-0.5, -0.5), True))

    win.showMaximized()
    print()
    print("THE WINDOW IS LIVE — your hands, in order:")
    print("  1. Click the Subtract row (CSG) in the feature tree.")
    print("  2. In the property panel, set offset_z to -1.0 "
          "(the placement edit — ONE rebuild).")
    print("  3. The tree's Fillet row reads FIL!.")
    print("  4. Click the FIL! row: the panel's REFUSAL row carries the")
    print("     payload (reference_vanished, why, remedy); Counts (V/E/F/H)")
    print("     stays 14/20/9/1 — the last good body kept.")
    print("  5. No crash; the CSG row healthy (the square through hole).")
    try:
        input("\nPress Enter here to close the window... ")
    except EOFError:
        pass
    win.close()


if __name__ == "__main__":
    main()
```

**The maintainer's acts on the live window:**

1. Click the **Subtract** row (`CSG`) in the feature tree.
2. In the panel, set **offset_z** to **−1.0** (the placement edit).

**Expected — ONE rebuild, the death shown, nothing mute:**
- The tree's fillet row reads **`FIL!`** (the visible refusal state,
  the `CSG!` pattern extended).
- Clicking the `FIL!` row, the panel's **REFUSAL** row carries the
  resolver's payload, verbatim:

  > `reference_vanished: sub#4:isect[box#1:top × ext#3:lat(0)]@hi — why: the authored entity was replaced; remedy: reselect the surviving entity by its new name (the new generation's query surfaces carry it), or revert the edit.`

- **Counts (V/E/F/H)** stays **`14/20/9/1`** and **Volume**
  **`6.77505976 (exact)`** — the last good body KEPT, never a mute
  body; **Edge reference** still the rim string.
- The CSG row is healthy — the operand re-materialized as the square
  through hole (no ERROR row); no crash anywhere.

**Sources:** the register's m5 row ("LA MORTE (classe 4) — the
sketch re-author … plus the placement edit from the panel: ONE
rebuild … the resolver's payload SHOWN — reference_vanished, WHAT
died (the row, named), WHY (the authored entity was replaced), the
remedy (reselect, or revert) — the last good body KEPT (14/20/9),
the tree row FIL!, the panel's REFUSAL row on selection, never a
crash, never a mute body"); the m5 UI test
(bindings/python/tests/test_ui_fillet_workbench.py, test 5); the m4
row's counterproof (the quartet's own scenario).

## Step 8 — Save v0.6, load, bit-identical, the edit still silent

1. *(The save already happened at the end of step 6 — the good state
   in `m6_blind_fillet.CAD_0`, declaring `"version": "0.6.0"`).*
2. After the death session, launch the workbench again
   (`PYTHONPATH=build/python python -m CAD_0.ui`); **File → Open…**
   the saved file.

**Expected — the body bit-identical:** the tree whole (`Box` SDF,
`Sketch`, `Extrude` FEAT, `Subtract (Box#1 - Extrude#3)` CSG,
`Fillet_5` FIL — the same ids, the same names); clicking the FIL row
the panel answers exactly as step 6 left it: **Edge reference** = the
rim string verbatim, **Counts (V/E/F/H)** = `14/20/9/1`, **χ** = `2`,
**Volume** = `6.77505976 (exact)`, the same **Born face(s)**/
`Born edge names` rows (the m4 row: the round-trip's digests N=10,
every cycle equal to the first — the first re-save IS the golden
bytes).

3. The edit, still silent: set the **radius** editor to **0.125**.

**Expected:** no dialog, no re-selection — the re-run answers
`reference_resolved` (the status bar: "Set radius = 0.125 — the
feature re-ran on the new radius (the reference taken once)");
**Counts (V/E/F/H)** stays **`14/20/9/1`** (fixed at 0.125 as at
0.25 — the m4/m5 rows' own pin); **Volume** = **`6.81078026
(exact)`** — the closed form's own value at the new r; the reference
row unchanged. **Edit → Undo** once: 0.25 and `6.77505976` back;
**Edit → Redo**: 0.125 and `6.81078026` — bit-identical.

**Sources:** the register's m4 row (THE RE-RUN AFTER THE LOAD —
"the radius edit ONE scene mutation answering reference_resolved …
the counts fixed for every in-band r (14/20/9 at 0.125 as at 0.25)",
the round-trip's digests, the corpus `native_v0_6_0.CAD_0`); the m5
row (the edit silent); the volume at 0.125 the D5 closed form at the
new r ((8−0.375π)−V_r(0.125) ≈ 6.810780258845277).

---

## The sources, in one list

- **ADR-0039 D6, the m6 rung** — docs/adr/0039-the-fillet.md, the
  morsi list item 5 (lines 823-829): the authority for the exit's
  shape.
- **ADR-0039 D5** (lines 656-675): the two guide cases pre-pinned —
  the box (10/15/7/χ=2, 7.5+π/8, the edge `box#1:vertical2`) and the
  rim (the reference string, 14/20/9/H=1/χ=2, the volume and its
  closed form, the centroid on the axis).
- **The register's Phase G row** (docs/status/README.md): the m2 row
  (the box golden, the names, the edit contract, the closed form),
  the m3 row (the rim golden, the names, the census, the volume), the
  m4 row (the v0.6.0 rebased bump, the recipe verbatim, the tavola's
  lodging, the round-trip's digests, the re-run after the load, the
  corpus), the m5 row (the selection character for character, the
  refusals with the bounds in the words, the silent edit, the death
  shown, the report saying everything, the six tests).
- **The goldens**: tests/unit/brep/test_g2_u3_m2_fillet.cpp,
  tests/unit/brep/test_g2_u3_m3_toroidal.cpp,
  tests/golden/native_v0_6_0.CAD_0, and the m5 UI suite
  (bindings/python/tests/test_ui_fillet_workbench.py).
- **The G1 precedent**: the register's Phase G row and cronaca — "G1
  COMPLETE AND CLICK-TESTED (2026-10-26) … the D7.5 table read in
  the report, the volume verified by hand, the radius edited live".

Every expected value above was verified on the local build at
`027efd6` through the same doors the workbench's widgets fire (the
scene API and the report module); the UI wiring itself is the m5
suite's, green in CI on the full matrix.
