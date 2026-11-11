// core/brep/src/survival_ledger.cpp
//
// K1 m6 — the survival ledger's own door (see survival_ledger.hpp
// for the contract; this file is the walk, the discipline, and the
// arithmetic — nothing else). The resolver is the ONLY opinion on
// the classes: this door builds the generations the caller's
// evidence names, consumes resolve()'s answers, and enforces the
// metric's own discipline (a death without a name is refused; the
// fraction never without the register).
#include "CAD_0/brep/survival_ledger.hpp"

#include <stdexcept>

namespace CAD_0::brep {

const char* survival_edit_token(SurvivalEditKind k) noexcept {
    switch (k) {
    case SurvivalEditKind::SilentRadiusEdit:
        return "silent_radius_edit";
    case SurvivalEditKind::SketchRootEdit:
        return "sketch_root_edit";
    case SurvivalEditKind::LoadBooleanRerun:
        return "load_boolean_rerun";
    case SurvivalEditKind::ChainedFilletEdit:
        return "chained_fillet_edit";
    case SurvivalEditKind::RecipeRerun:
        return "recipe_rerun";
    }
    return "unknown";
}

namespace {

// The walk's own state for one reference — the current identity (a
// rename follows the continuer), the liveness (a death stops the
// walk; an ambiguity suspends it), and the class-so-far.
struct WalkState {
    geometry::ShapeId current{};
    bool alive{true};
    bool has_class{false};
    SurvivalEntry entry{};
};

[[noreturn]] void refuse(const std::string& what) {
    throw std::invalid_argument("survival_ledger: " + what);
}

// The death's own naming — the discipline's single point. `step` and
// `reference` are the caller's own evidence; `r` is the resolver's
// answer (the row carried whole, the cause verbatim). The CLASS is
// the caller's own decision (the edit's answer or the death itself)
// — this fills the death's payload alone.
void fill_death(SurvivalEntry& e, const Resolution& r,
                const SurvivalStep& step,
                const SurvivalReference& ref) {
    // WHAT died: the row's own render, the reference's when the row
    // reads honestly unnamed (vertices, the queued rows — the
    // reference IS what died, its render its name).
    e.died = r.vanished.row.name.empty() ? ref.name
                                         : r.vanished.row.name;
    e.cause = r.vanished.why;
    e.remedy = r.vanished.remedy;
    e.killer = step.killer;
    e.step = step.name;
    e.dead = true;
    // THE DISCIPLINE — D1.4's own words made a refusal: "a death
    // without its class name is a crash". Any of the four names
    // empty and the ledger REFUSES to register the member: never a
    // partial ledger, never an anonymous death riding a fraction.
    if (e.died.empty() || e.killer.empty() || e.cause.empty() ||
        e.step.empty()) {
        refuse("a death without its name is refused - the member '" +
               e.member + "' observed the reference '" + ref.name +
               "' die at step '" + step.name + "' without its full "
               "name (what: '" + e.died + "', killer: '" + e.killer +
               "', cause: '" + e.cause +
               "'). A death without its class name is a crash "
               "(ADR-0041 D1.4). Remedy: declare the step's killer "
               "(the consuming feature's own name) and carry the "
               "tavola's vanished row with its cause verbatim — the "
               "ledger registers only named deaths.");
    }
}

} // namespace

std::string SurvivalLedger::deaths_text() const {
    // One line per death, the register's own columns: WHERE the
    // member, WHAT the reference, WHO the killer, WHY the cause,
    // the step it dies at. The remedy rides the line's tail — the
    // actionable answer, never a bare row.
    std::string out;
    for (const SurvivalEntry& d : deaths) {
        out += d.member;
        out += " | ";
        out += d.reference;
        out += " | ";
        out += d.killer;
        out += " | ";
        out += d.cause;
        out += " | ";
        out += d.step;
        out += " | remedy: ";
        out += d.remedy;
        out += '\n';
    }
    return out;
}

std::string SurvivalLedger::text() const {
    // The report's own render: the members' rows first (the kind,
    // the reference, the class's code — the rename's from->to and
    // the map declared beside a RENAMED, the candidates beside an
    // AMBIGUOUS), then the register, then the fraction's own pair —
    // the three things the metric demands, in one comparable string
    // (deterministic: strings and integers only).
    std::string out;
    for (const SurvivalMemberLedger& m : members) {
        out += "member '";
        out += m.name;
        out += "' (";
        out += survival_edit_token(m.kind);
        out += ")\n";
        for (const std::string& seat : m.seats_declared) {
            out += "  seat declared: ";
            out += seat;
            out += '\n';
        }
        for (const SurvivalEntry& e : m.entries) {
            out += "  ";
            out += e.reference;
            out += " -> ";
            out += e.code;
            if (e.kind == Resolution::Kind::Renamed) {
                out += " (";
                out += e.renamed_from;
                out += " -> ";
                out += e.renamed_to;
                out += ", map: ";
                out += e.renamed_map;
                out += ")";
            }
            for (const std::string& c : e.candidates) {
                out += " candidate: ";
                out += c;
            }
            out += '\n';
        }
    }
    out += "deaths: ";
    out += std::to_string(deaths.size());
    out += '\n';
    out += deaths_text();
    out += "survival fraction: ";
    out += std::to_string(resolved);
    out += "/";
    out += std::to_string(total);
    out += '\n';
    return out;
}

SurvivalLedger survival_ledger(
    const std::vector<SurvivalMember>& members) {
    SurvivalLedger ledger;

    for (const SurvivalMember& m : members) {
        if (m.name.empty())
            refuse("a member without a name - the report cannot name "
                   "what it cannot call. Remedy: name the member "
                   "(the corpus's own render, e.g. 'the silent "
                   "radius edit r=0.125').");
        if (m.references.empty())
            refuse("the member '" + m.name +
                   "' tracks no reference - a survival ledger over "
                   "nothing is a vanity number (ADR-0041 D1.4's own "
                   "antidote). Remedy: pin the member's references "
                   "(the baseline's own, or the member's declared "
                   "seats).");
        for (const SurvivalReference& r : m.references) {
            if (r.name.empty())
                refuse("the member '" + m.name +
                       "' carries a reference without a render - the "
                       "verdict's own subject must be named. Remedy: "
                       "the pinned string (the grammar's render, "
                       "character for character).");
            if (r.grammar == nullptr)
                refuse("the reference '" + r.name + "' (member '" +
                       m.name + "') carries no grammar - the "
                       "resolution reads the CREATING operation's "
                       "tables (ADR-0036 D3's per-creator rule). "
                       "Remedy: pass the declaration the reference "
                       "was rendered under.");
        }

        SurvivalMemberLedger out;
        out.kind = m.kind;
        out.name = m.name;
        out.seats_declared = m.seats_declared;
        out.digest = m.digest;

        // The walk's state, one per reference.
        std::vector<WalkState> walk(m.references.size());
        for (std::size_t i = 0; i < m.references.size(); ++i) {
            walk[i].current = m.references[i].id;
            walk[i].entry.member = m.name;
            walk[i].entry.reference = m.references[i].name;
        }

        for (const SurvivalStep& step : m.steps) {
            for (const SurvivalStepRef& sr : step.refs) {
                if (sr.reference >= m.references.size())
                    refuse("the step '" + step.name + "' (member '" +
                           m.name + "') addresses reference " +
                           std::to_string(sr.reference) +
                           " - the member pins " +
                           std::to_string(m.references.size()) +
                           ". Remedy: fix the index (a caller bug).");
                WalkState& w = walk[sr.reference];
                if (!w.alive)
                    refuse("the step '" + step.name + "' (member '" +
                           m.name + "') resolves the reference '" +
                           m.references[sr.reference].name +
                           "' after its death - the walk stopped at "
                           "the death. Remedy: drop the stale entry "
                           "(the dead do not resolve).");
                if (sr.old_tavola == nullptr || sr.new_body == nullptr)
                    refuse("the step '" + step.name + "' (member '" +
                           m.name + "') hands the reference '" +
                           m.references[sr.reference].name +
                           "' an incomplete generation - the old "
                           "side needs its tavola, the new side its "
                           "body. Remedy: derive both (the tavola "
                           "where the reference's rows live, the "
                           "step's own result body).");

                const SurvivalReference& ref =
                    m.references[sr.reference];
                // The generations: the old side's tavola (the rows
                // the reference lives in), the new side's body, the
                // reference's own grammar on both (the per-creator
                // rule — the caller holds the declaration).
                const Generation old_gen{nullptr, sr.old_tavola,
                                         ref.grammar, ref.multi_chain};
                const Generation new_gen{sr.new_body, nullptr,
                                         ref.grammar, ref.multi_chain};
                const Resolution r =
                    resolve(w.current, old_gen, new_gen, sr.edit);

                switch (r.kind) {
                case Resolution::Kind::Resolved:
                    // The class's own answer on an EDIT step; the
                    // kept-through-the-consumer continuation on a
                    // MEAL step (the class untouched — the feature's
                    // act is not the edit's).
                    if (step.kind == SurvivalStepKind::Edit) {
                        w.entry.kind = r.kind;
                        w.entry.code = resolution_code(r.kind);
                        w.has_class = true;
                    }
                    break;
                case Resolution::Kind::Renamed:
                    // The continuer followed: the reference's own
                    // next step (D3.2 — "the reference follows to
                    // the new name"), the map declared on the
                    // payload, never silent.
                    w.entry.kind = r.kind;
                    w.entry.code = resolution_code(r.kind);
                    w.entry.renamed_from = r.renamed.from_name;
                    w.entry.renamed_to = r.renamed.to_name;
                    w.entry.renamed_map = r.renamed.map;
                    w.has_class = true;
                    w.current = r.renamed.to;
                    break;
                case Resolution::Kind::Ambiguous:
                    // The reference does not guess — the ledger does
                    // not either: the candidates named, the walk
                    // suspended.
                    w.entry.kind = r.kind;
                    w.entry.code = resolution_code(r.kind);
                    w.entry.candidates = r.ambiguous.candidate_names;
                    w.has_class = true;
                    w.alive = false;
                    break;
                case Resolution::Kind::Vanished: {
                    // The death: FULLY named, whatever step it lands
                    // on — the register's own payload. The CLASS is
                    // the edit's answer when the edit already gave
                    // one (the meal is the feature's own act, not
                    // the edit's — the death rides the register
                    // BESIDE the class, it does not overwrite it);
                    // on an edit step — or a meal that precedes any
                    // edit step — the death IS the class (the edit
                    // killed the reference).
                    const bool death_is_the_class =
                        step.kind == SurvivalStepKind::Edit ||
                        !w.has_class;
                    fill_death(w.entry, r, step, ref);
                    if (death_is_the_class) {
                        w.entry.kind = Resolution::Kind::Vanished;
                        w.entry.code =
                            resolution_code(w.entry.kind);
                    }
                    w.has_class = true;
                    w.alive = false;
                    break;
                }
                case Resolution::Kind::ClosureDefect:
                    // The guard — the evidence's tavola is defective:
                    // the member is refused, the detail verbatim (the
                    // resolver's own words, D2's floor).
                    refuse("the evidence's tavola is defective - the "
                           "reference '" +
                           ref.name + "' (member '" + m.name +
                           "', step '" + step.name +
                           "') finds no row at all: " +
                           r.closure_defect.detail +
                           " Remedy: carry the tavola the reference "
                           "was made against (the creating or the "
                           "consuming operation's own).");
                }
            }
        }

        // The entries: every reference's walk closes with a class
        // (the edit step's own answer — the input contract below
        // demands one) and the member's arithmetic composes.
        for (std::size_t i = 0; i < walk.size(); ++i) {
            if (!walk[i].has_class)
                refuse("the reference '" + m.references[i].name +
                       "' (member '" + m.name +
                       "') walks no EDIT step - the class the member "
                       "reports is the EDIT's own answer (the "
                       "metric's question: does the reference "
                       "survive the edit?). Remedy: give the "
                       "reference its edit step (the "
                       "re-materialized chain's generations).");
            out.entries.push_back(std::move(walk[i].entry));
            const SurvivalEntry& e = out.entries.back();
            // THE NUMERATOR, pinned where it counts (ADR-0041 D1.4:
            // "references that resolve class 1 ... over the total"):
            // class 1 ALONE enters it — MATCH is the numerator;
            // RENAMED, AMBIGUOUS and VANISHED ride the denominator
            // and the register, never the numerator.
            switch (e.kind) {
            case Resolution::Kind::Resolved: ++out.resolved; break;
            case Resolution::Kind::Renamed: ++out.renamed; break;
            case Resolution::Kind::Ambiguous: ++out.ambiguous; break;
            case Resolution::Kind::Vanished:
                ++out.vanished;
                break;
            case Resolution::Kind::ClosureDefect: break;  // refused
            }
            // THE REGISTER — the fraction's own composition: every
            // death the walk observed (the meals included — the
            // feature's own acts, m5's census lesson) rides the
            // ledger beside the counts, never behind them. The
            // discipline's guarantee: an anonymous death never
            // reached here — the door refused the member first.
            if (out.entries.back().dead)
                ledger.deaths.push_back(out.entries.back());
        }

        ledger.total += out.entries.size();
        ledger.resolved += out.resolved;
        ledger.renamed += out.renamed;
        ledger.ambiguous += out.ambiguous;
        ledger.vanished += out.vanished;
        ledger.members.push_back(std::move(out));
    }
    return ledger;
}

} // namespace CAD_0::brep
