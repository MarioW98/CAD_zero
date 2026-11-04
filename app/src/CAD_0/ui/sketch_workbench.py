"""CAD_0.ui.sketch_workbench — the solver session (ADR-0040 D4, G4 m3).

THE WORKBENCH DERIVED, NOT PAINTED. The solver landed in m2 with its
whole surface queryable from Python (the H2-u6 discipline); this
module is the CONSUMER that turns it into the interface's state:

  * the authored CONSTRAINT LIST — session state in m3. The format
    bump is NOT this morso's to take (the parallel discipline: m4
    owns the serialization — the constraints as FIELDS, never raw
    structs, the padding discovery paid — and the register arbitrates
    the version number against the fillet line). The list lives for
    the lifetime of the editing session and is declared as such:
    saving the project drops it, by design, until m4 lands.
  * LA BOLLETTA — the honest DoF counter, recomputed at the COLD
    SITES only (a constraint added/removed/re-valued, a parameter
    write, a pin, an entity write): the cache below is the whole
    cost model, zero at rest — the certificate's own economy. The
    counter is the BILL'S OWN TEXT (``describe()``), never a UI-side
    arithmetic that could disagree with the graph.
  * THE CLASSES IN WORDS — solved / sketch_not_solvable /
    sketch_over_constrained / sketch_degenerate: the register's own
    vocabulary, the payloads quoted from the kernel's records (the
    admission's message VERBATIM, the diagnosis VERBATIM, the
    culprit's label VERBATIM). No paraphrase that invents, ever.

THE SCENE IS THE TRUTH. The session never caches geometry: every
cold act rebuilds the model from the recipe's CURRENT chains (the
single construction route — the kernel factories, the same one the
canvas, the panel and the main window build entities through) and
re-admits the authored list in order. The m2's padding lesson is
practiced here by construction: no raw struct crosses this module,
only kernel-built objects and their dict form.
"""

from __future__ import annotations


# The classes' names — the error taxonomy's own (ADR-0040 D2.2, the
# sketch_* family beside fillet_* and reference_*). Quoted constants,
# never recomputed: the words the register writes are the words the
# interface says.
CLASS_SOLVED = "solved"
CLASS_NOT_SOLVABLE = "sketch_not_solvable"
CLASS_OVER_CONSTRAINED = "sketch_over_constrained"
CLASS_DEGENERATE = "sketch_degenerate"


def _scene_mod():
    import CAD_0
    return CAD_0.scene


def entities_from_dicts(chain_dicts):
    """A chain's dict entities as kernel objects — through the
    factories (the single construction route; the dict form is the
    reader, the factory is the writer, the values round trip
    float32-exact because both sides speak the recipe's floats).

    No struct is built by hand and no field is re-derived: the arc's
    (start_angle, sweep) pass through verbatim — the a1 fold the
    panel's parameter surface practices is NOT taken here (the
    solver's stored output is written back as the solver produced
    it)."""
    s = _scene_mod()
    ents = []
    for e in chain_dicts:
        if e["kind"] == "segment":
            ents.append(s.sketch_segment(
                (e["a"][0], e["a"][1]),
                (e["b"][0], e["b"][1]), e["interior_left"]))
        elif e["kind"] == "circle":
            ents.append(s.sketch_circle(
                (e["center"][0], e["center"][1]),
                e["radius"], e["hole"]))
        else:
            ents.append(s.sketch_arc(
                (e["center"][0], e["center"][1]),
                e["radius"], e["hole"],
                e["start_angle"], e["sweep"]))
    return ents


def native_chains(scene, node_id: int):
    """The recipe's chains as native SketchChain objects — the model
    constructor's own input, rebuilt from the truth at every cold
    act."""
    s = _scene_mod()
    return [s.SketchChain(entities_from_dicts(chain))
            for chain in scene.sketch_chains(node_id)]


def field_value(e: dict, name: str) -> float:
    """A field's CURRENT value on the recipe's dict form, by the
    kernel's own field name (sketch_field_names' vocabulary): the
    value the pin consumes at the moment of the gesture (D4 — the
    parameter equals the value it had, never a rounded copy)."""
    if name in ("x0", "y0"):
        return float(e["a"][0 if name == "x0" else 1])
    if name in ("x1", "y1"):
        return float(e["b"][0 if name == "x1" else 1])
    if name in ("cx", "cy"):
        return float(e["center"][0 if name == "cx" else 1])
    if name == "r":
        return float(e["radius"])
    if name == "a0":
        return float(e["start_angle"])
    if name == "sweep":
        return float(e["sweep"])
    raise KeyError("not a recipe field name: {}".format(name))


def rebuild_with_value(c, value: float):
    """A value-carrying constraint rebuilt at a new value — through
    the FACTORY (the value rules are the factories' own: v > 0 for
    the named dimensions, any finite v for the pin; an illegal value
    raises the kernel's remedy). D3.3's re-value as the honest route:
    same operands, same kind, one number different."""
    s = _scene_mod()
    kind = str(c.kind)
    if kind == "length":
        return s.constraint_length(c.ea, value)
    if kind == "radius":
        return s.constraint_radius(c.ea, value)
    if kind == "distance":
        return s.constraint_distance(c.a, c.b, value)
    if kind == "pin":
        return s.constraint_pin(c.pin, value)
    return None


class SketchSolverSession:
    """One sketch node's constraint state: the authored list, the
    cached bill, the cached words. The CANVAS owns the write routes
    (every mutation lands inside a command on the shared stack — the
    single history); this session is the state those commands mutate
    and the bridge to the solver the m2 landed.

    The re-admission policy, declared: the authored list is re-admitted
    in order at every cold act, at the CURRENT geometry (the canonical
    point — the same rank test the add itself passed, honestly re-run).
    A constraint that no longer raises the rank at the moved point is
    REPORTED in the words (the admission's own message, verbatim) and
    left out of the working model; the authored list itself is never
    silently rewritten — the state is an answer, never a silence. In
    that state the SOLVE refuses to store (the recipe untouched): a
    store from a reduced model would let a constraint's effect drift
    away unreported.
    """

    def __init__(self, scene, node_id: int) -> None:
        self._scene = scene
        self._node_id = node_id
        self._constraints = []       # the authored list (kernel objects)
        self._bill_text = ""         # the counter's cache (cold sites only)
        self._feedback = ""          # the last solve/admission's words
        self.refresh_bill()

    # ----- the state the commands mutate (inside commands only) -----

    def constraints(self):
        return list(self._constraints)

    def constraint_count(self) -> int:
        return len(self._constraints)

    def constraint_at(self, index: int):
        if 0 <= index < len(self._constraints):
            return self._constraints[index]
        return None

    def admit(self, constraint):
        """The add: the admission's own record is the answer (never an
        exception, never a silence). On success the candidate is
        APPENDED to the authored list (append-only in v1, D3.3 — the
        list itself is the identity, the model's re-admission never
        rewrites it) and the counter refreshes — the cold site. On the
        refusal the words are the record's own message (the class, the
        culprit, the bill, the remedy — VERBATIM)."""
        model, _refused = self._model_with_list(self._constraints)
        record = model.add_constraint(constraint)
        if record.admitted:
            self._constraints.append(constraint)
            self._feedback = ""
        else:
            self._feedback = record.message
        self.refresh_bill()
        return record

    def remove_at(self, index: int) -> bool:
        """D3.3's remove (the v1 edit family: add, remove, re-value).
        The removal is a structural no-op for the rank invariant; the
        counter refreshes — the cold site."""
        if not (0 <= index < len(self._constraints)):
            return False
        del self._constraints[index]
        self._feedback = ""
        self.refresh_bill()
        return True

    def revalue_at(self, index: int, value: float) -> bool:
        """D3.3's re-value — the dimension edit: the constraint rebuilt
        at the new value THROUGH THE FACTORY (the value rules are the
        factories' own; an illegal value raises, the caller shows the
        remedy, the previous value stands). The counter does not move
        (the consumption is unchanged) but the act is cold: the cache
        refreshes."""
        if not (0 <= index < len(self._constraints)):
            return False
        self._constraints[index] = rebuild_with_value(
            self._constraints[index], value)
        self.refresh_bill()
        return True

    def reinsert(self, index: int, constraint) -> None:
        """The remove's undo half: the constraint back at its authored
        position (the position is the identity — the reinsert restores
        the list EXACTLY as it was, never a re-append)."""
        if 0 <= index <= len(self._constraints):
            self._constraints.insert(index, constraint)
        else:
            self._constraints.append(constraint)
        self.refresh_bill()

    def restore(self, constraints) -> None:
        """The solver-edit command's undo half: the whole constraint
        list back as it was (the objects themselves, captured by the
        command — the same kernel objects, never rebuilt)."""
        self._constraints = list(constraints)
        self.refresh_bill()

    def report_refusal(self, record) -> None:
        """A pre-admission's refusal reported (the canvas's scratch
        checks and the constraint commands' honest first voice): the
        record's message IS the words — sketch_over_constrained with
        the culprit, the bill and the remedy, VERBATIM."""
        self._feedback = record.message

    # ----- the bridge (the model rebuild discipline) -----

    def _model_with_list(self, constraints):
        """The fresh model: the recipe's CURRENT chains, the list
        re-admitted in order. Returns (model, refused_records) — the
        refused are the honest answers for constraints that no longer
        raise the rank at the moved geometry (reported, never dropped
        from the authored list). A constraint whose operand is out of
        range (a command's UNDO mid-flight: the entities already
        removed, the list not yet) is SKIPPED here — the state is
        transient by construction, the LIFO order guarantees the list
        matches the structure by the command's end; the authored list
        is never rewritten by a rebuild."""
        s = _scene_mod()
        model = s.SketchSolveModel(native_chains(self._scene,
                                                 self._node_id))
        refused = []
        for c in constraints:
            try:
                record = model.add_constraint(c)
            except (ValueError, RuntimeError):
                continue  # the operand is gone — a transient mid-command state
            if not record.admitted:
                refused.append(record)
        return model, refused

    def model(self):
        """The working model for the CURRENT state — the graph the
        counter's number must confirm (the tests' own voice)."""
        model, _refused = self._model_with_list(self._constraints)
        return model

    def native_chains_now(self):
        """The recipe's chains as native objects (the constraint
        renders' own operand tables)."""
        return native_chains(self._scene, self._node_id)

    # ----- LA BOLLETTA (the honest counter) -----

    def refresh_bill(self) -> None:
        """The cold-site recomputation — the ONLY place the counter
        moves. The bill is the working model's own (the entities' DoF
        minus the ADMITTED consumptions — a refused re-admission shows
        in the count, honestly); the text is the bill's own
        describe(), so the interface and the graph cannot disagree by
        construction."""
        model, _refused = self._model_with_list(self._constraints)
        self._bill_text = model.bill().describe()

    def bill_text(self) -> str:
        """The cached counter (e.g. '16 - 13 = 3 free', '12 - 12 = 0
        (FULL)') — the reading at rest costs nothing."""
        return self._bill_text

    # ----- THE CLASSES IN WORDS (the solve's feedback) -----

    def feedback_text(self) -> str:
        """The last act's words: the class name in the register's own
        vocabulary with the kernel's payload quoted verbatim. Empty
        after a plain successful admission (nothing to report — the
        counter already speaks)."""
        return self._feedback

    def words_for_result(self, result) -> str:
        """The solve's three honest classes as the interface's words
        (D4): solved with the free count; sketch_not_solvable with
        the residual and THE NAMED CULPRIT and the diagnosis;
        sketch_degenerate with the kernel's own message VERBATIM."""
        s = _scene_mod()
        if result.status == s.SketchSolveStatus.solved:
            return "{} — {}".format(CLASS_SOLVED, result.bill.describe())
        if result.status == s.SketchSolveStatus.degenerate:
            return "{} — {} ({})".format(
                CLASS_DEGENERATE, result.degenerate_message,
                result.degenerate_entity)
        # sketch_not_solvable: the residual, the largest violation's
        # own label (the culprit named), the diagnosis VERBATIM.
        largest = result.violations[result.largest]
        return ("{} — the largest violation {:.6g} on {}; |F| = "
                "{:.6g}; {}; the values at the exit are the bounded "
                "closest approach, the recipe untouched").format(
                    CLASS_NOT_SOLVABLE, abs(largest.value), largest.label,
                    result.residual_norm, result.diagnosis)

    def solve(self):
        """IL SOLVE on the current state: the model's own answer plus
        the refused-re-admission records (the caller stores back ONLY
        when the list is empty — a store from a reduced model would
        let a constraint's effect drift away unreported). The words
        cache from whichever honest answer comes first."""
        model, refused = self._model_with_list(self._constraints)
        result = model.solve()
        if refused:
            self._feedback = refused[0].message
        else:
            self._feedback = self.words_for_result(result)
        return model, result, refused

    def store_solved(self, model) -> None:
        """The solved chains written back to the recipe — per entity,
        through the factories (the kernel's construction checks run
        on the solver's output; the values are the stored float32 the
        model holds, verbatim — no a1 fold, no struct copy)."""
        for ci, chain in enumerate(model.chains_dump()):
            for ei, e in enumerate(entities_from_dicts(chain)):
                self._scene.sketch_set_entity(self._node_id, ci, ei, e)

    # ----- the panel's constraint rows (the registry pattern) -----

    def constraint_rows(self, chain: int, index: int):
        """The selected entity's value-carrying constraints as the
        panel's rows: ({label: value}, {label: the authored index}).
        The label is the constraint's OWN render (describe_constraint,
        verbatim — the same words a refusal would quote); the value is
        the constraint's value (a float editor — the re-value route).
        The non-value kinds have no editable row (their remedy surface
        is the remove)."""
        s = _scene_mod()
        chains = self.native_chains_now()
        rows = {}
        registry = {}
        for i, c in enumerate(self._constraints):
            kind = str(c.kind)
            binds = False
            if kind in ("length", "radius"):
                binds = (c.ea.chain == chain and c.ea.entity == index)
            elif kind == "pin":
                binds = (c.pin.entity.chain == chain and
                         c.pin.entity.entity == index)
            elif kind in ("coincidence", "distance"):
                binds = ((c.a.entity.chain == chain and
                          c.a.entity.entity == index) or
                         (c.b.entity.chain == chain and
                          c.b.entity.entity == index))
            elif kind in ("parallel", "perpendicular", "tangent"):
                binds = ((c.ea.chain == chain and c.ea.entity == index) or
                         (c.eb.chain == chain and c.eb.entity == index))
            if not binds or kind not in ("length", "radius", "distance",
                                         "pin"):
                continue
            label = s.describe_constraint(c, chains)
            rows[label] = float(c.value)
            registry[label] = i
        return rows, registry

    def describe_all(self):
        """Every constraint's own render, authored order — the sketch
        node's panel row (the list the interface shows, the register's
        words)."""
        s = _scene_mod()
        chains = self.native_chains_now()
        return [s.describe_constraint(c, chains)
                for c in self._constraints]
