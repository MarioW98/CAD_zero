# ADR-0038: G2 unit 2 — the extended booleans, act two: the harvest, the blind and the crossed, the cap insertion derived

## Status

**Proposed (2026-11-02) — docs only: no code moves, no golden moves, no
format moves in this delivery.** The morsi 2+ implement only on the
maintainer's review of THIS document — the STOP the order pins: the ADR
passes the REVISIONE before a morsel of code exists.

What this ADR is: **act two of the extended booleans** — the unit
ADR-0033 D5.2 promised as G2's second rung, re-derived from zero on the
foundations H1 and H2 rebuilt while the first act lay frozen on a dead
branch. The first act is harvested here, never merged: **IL RACCOLTO**
(the inventory, below) records what survives of it as truth and what
dies as code, with the lesson dated. Act two's scope is derived, not
inherited: the subtract's registered classes — the ones ADR-0037 D4
queued to G2 by the code's own twice-written pointer
(subtract.hpp:1183-1185, :1148-1149) — land first, because they are the
walls with pinned refusals in today's tree: **IL FORO CIECO** (the
BLIND: the internal cap — the SSI-driven cap insertion),
**LO SBORDO** (the CROSSED: the non-annular pierced face — the
re-authored outer loop), and **LA PARETE-NEL-PIANO** (the atrium of
ADR-0037 D3.6 — derived here to its honest seat, which is NOT this
unit's). The sphere booleans of the old act one — the promise D5.2
worded — are re-queued by name with the harvest's derivations traveling
to their consumer (D4).

The method is the house's own, every clause citing its mother: the
derivation is the authority (ADR-0033's acceptance record); MAI "tutte"
— the scope is what the guide cases and the fillet REALLY reach
(ADR-0031 D5, applied as ADR-0036/0037 D4 applied it); the goldens are
pre-pinned BEFORE the code with the derivation visible (ADR-0035 D5 /
ADR-0036 D5 / ADR-0037 D5, the protocol verbatim); the register surgery
is declared here and applies in the same act as acceptance (ADR-0035
D6's protocol); the tolerances are drawn from the existing typed worlds
— **zero new constants** (ADR-0035 D2's vocabulary is complete for this
unit's needs, and the one open question it left — ε_line, ADR-0037
D2's sentinel — is DECIDED here: D2, with the reason written).

Daughter of ADR-0033 (the G2 boundary and the ladder this unit
re-orders honestly), of ADR-0031 D4.2/D5 (the boolean guarantees and
the one-guide-case scale), of ADR-0035 (the typed tolerance worlds every
band here reads), of ADR-0036 (the naming grammar D3 extends, the
descent vocabulary D3's tavole consume, the walk-tables vocabulary gap
THIS unit pays — the morse registers' own "yet"), of ADR-0037 (the
coplanar class: D4's queue hands this unit its classes; D2's sentinel
question decided here; D3.6's gift forwarded untouched, with the
correction the derivation forces — D1.3), of ADR-0023 (the seam
conventions the derived counts apply), of ADR-0032 (the Layer-2
provenance contract D2's ε_line decision re-reads; NOT amended — the
reason below), and of the G1 code itself (subtract.hpp's walls, cited
line by line in D1 — the repo is the authority).

## IL RACCOLTO — the old branch's harvest (the inventory, dated)

**The story, verified against the log.** `origin/G2` forked from main
at 3615453 (G2 unit 1: the SSI table, 2026-10-26 16:43) and froze at
52915c0, **2026-10-29 15:13:30 +0100** — two commits of its own:

1. **a62f564 (2026-10-29 15:07:17)** — "Commit 1/4 — c851bdc": the H1
   landing squashed onto the branch, so the work could continue on the
   remediated arithmetic. Its own body says it plainly: *"G2 unit 2
   stays paused on top of this foundation."*
2. **52915c0 (2026-10-29 15:13:30 — six minutes later)** — "G2
   avanzamento booleani esatti": the extended booleans, act one. 1,902
   lines of `booleans_ext.hpp`/`.cpp`, a rewritten `ssi.cpp`, a heavily
   reworked `subtract.hpp`, new bindings and tests — and the register
   fight, below.

**What survives as TRUTH** (read, judged, cashed in by this ADR and by
the queue's consumers — never a byte merged):

- **The counts, verbatim from the act's own header**
  (booleans_ext.hpp:38-76, the branch's only honest artifact — the
  derivations were done before the code, the house method even there):
  the sphere×sphere family — union/lens 4V/4E/2F χ=2, bitten
  2V/2E/2F χ=2, the volumes by the classical cap formulas
  V₁+V₂−cap(R₁,R₁−h)−cap(R₂,R₂−D+h); the coaxial circle-prism family —
  bead 4V/6E/2F χ=0 genus 1, rivet 8V/11E/5F χ=2, core 6V/7E/3F χ=2,
  the core volume πr²·2w + 2·cap(R, R−w); the truncation family —
  far/near cap 2V/2E/2F χ=2, dome 11V/15E/7F/1H χ=2, slab 6V/6E/4F
  χ=2; and the two-holes second cut — 16/22/8/4, χ=−2 = 2(1−2), genus
  2, volume abc − 2πr²c. These are the golden expectations the
  re-derivation on the redeemed foundations must meet again — quoted
  there, re-derived there, never copied.
- **The XOR azimuth convention** (the re-frame's orientation care: the
  cylinder pair's negated azimuth, the sphere pair's shared one) — the
  derivation survives; its consumer is the sphere booleans' unit.
- **The degenerate dispositions**: the co-radial dedup
  (union/intersect keep verbatim, subtract refuses the empty); the
  external/internal tangencies; nested/separate; the Viviani wall; the
  frame budget's two-family refusal (its canonical example); the
  unequal-radii parallel pair honestly named (the transverse case
  typable, admitted by the criterion, awaiting its consuming unit —
  never "untypable"); and the two-holes' **peanut** — the overlapping
  circles' inner region is non-annular: a wall met and understood, the
  typed vocabulary's own convexity limit, refused by name.
- **The walls it left standing** — the honest part: the BLIND, the
  TANGENT and the CROSSED refusals untouched (verified in the diff: the
  branch never attempted them; its unit-2 scope was the sphere
  booleans). They were G2's by the code's own pointers, and act two
  derives them here.

**What dies as CODE** (never a byte merged — the maintainer's decision,
ferma):

- **The whole booleans_ext layer**: `float tol` and `Vec3f` at every
  signature — the pre-H1 world H1 demolished (ADR-0034: double
  throughout the exact path); one tolerance for every decision — the
  world ADR-0035's taxonomy demolished; positional identities only (no
  names, no tavola, no descent) — the world ADR-0036 built; the entry
  contract without the re-frame rule as G2-u1 landed it.
- **The ssi.cpp revert**: the branch un-did H1 in the table itself —
  `float radius`, `norm_two_pi(float)`, the double→float latitude
  boundary removed. Dead wholesale.
- **The register fight — the branch's gravest act, and the lesson.**
  The same commit that built the booleans DELETED ADR-0034 (258 lines),
  REMOVED ADR-0014's H1 amendment (the display-boundary quantizer), and
  REVERTED the H1 correction blocks in ADR-0031/0032 — the code, unable
  to comply with the record, rewrote the record instead. The register
  won: main kept every correction, and the branch froze with the fight
  unwon. The bytes of that fight die with the branch; the lesson
  survives, dated: **the code fought the record; the record won.** The
  pause (10-29 → 11-02) was not lost time — it is an experiment already
  paid for: the four days bought H2's taxonomy, the naming, the
  coplanar door, and this ADR cashes the check. The branch is
  archaeology: SI LEGGE, NON SI FONDE — read, never merged into the
  redeemed kernel.

## Context — where the exact path stands today (the walls, verified)

The G1 class (ADR-0032 D7, landed unit 4) runs end-to-end on its pinned
class: the convex prism tool, axis ⊥ the target's pierced pair, the
footprint strictly inside the pierced face, the span through. The
placement decision tree (subtract.hpp:1046-1191, the seat cited per
branch below) refuses every other placement with an actionable message,
each refusal pinned: the z-side miss and the SAT touch (the exact copy,
:1046-1057, :1080-1095), the SWALLOW (:1104-1110, pinned :1037), the
CROSSED (:1140-1152, pinned :952), the TANGENT (:1153-1169, pinned
:914), the BLIND (:1175-1186, pinned :973). The assembly copies A
verbatim, appends the walk tables' entities, the pierced faces gain
inner loops, the wall is re-windowed, the caps die. The counts are
pinned by the general formula (subtract.hpp:84-88): a chain tool of n
entities through a target (V_A, E_A, F_A) gives V = V_A+2n, E = E_A+3n,
F = F_A+n, H = 2 → χ = 0; the circle is the seam-wired special case
(4/5/1 on top of A's own). The naming (ADR-0036, landed H2-u6) covers
the classes' entities; the walk's other edges — the bottom/top curves,
the verticals, the rims — read honestly EMPTY outside the v1 extrude
vocabulary (extrude.hpp's own note), and "the joints' verticals have no
vocabulary row yet" (the morse registers' gap — THIS unit pays it,
D2/D3). The G2-u1 SSI table answers the four typed entries through the
bindings before any boolean consumes them; the re-frame rule and the
frame budget are landed; the same body twice serves the intra-body
pairs (ssi.hpp:229). The coplanar door (ADR-0037, landed H2-u5 m2) is
green with its goldens pinned — P4 8/12/6 χ=2, P8 20/32/14 χ=2, P11 A
whole, the ratifiche P1/P2 20/32/14 and P7/P10 16/26/12 — and the
exact-path classes ADR-0037 D4 queued to G2 stay queued: the
non-annular pierced face, the internal cap, the wall-in-plane.

## Decision

### D1. The taxonomy of the extension — the G1 walls, today's refusals cited line by line

#### 1. IL FORO CIECO (the BLIND) — the internal cap

*Today's refusal:* subtract.hpp:1175-1186 (pinned :973) — the tool
terminates inside the target: "the internal cap face class is G2's (it
needs the SSI-driven cap insertion, not the pass-through circle pair
G1 builds)". The z-band branch (:1172-1191) fires on `blind_lo ||
blind_hi` — a cap strictly between the pierced quotas.

*The class, derived.* The admitted family: **exactly one cap strictly
inside, the other side through** (z_lo ∈ (z_bot, z_top) within the
declared bands and z_hi ≥ z_top − tol, or the mirror). The result gains
the cavity's floor: **the internal cap — the tool's own cap at the end
of its course, carried into the result** with its plane and its
provenance (the stamp inherited, the record combined — the wall's own
pattern), re-oriented (the region bits flipped: the material is now
outside the profile — the G1 unit-4 convention applied to a cap). Its
rim is typed by the SSI table — the intra-tool pair, the entry
plane×cylinder ⊥ at the cap quota: the circle r at the axis∩plane,
`Circle2` on the cap's domain, `Line2` v=const on the wall's — "the
curves at cap quota from the G2-u1 table" (D2). The wall's window
re-times to [z_cap, z_top]; the pierced face at the through side gains
its inner loop exactly as G1 builds it — unchanged.

*The guide case:* the box [−1,1]³ with the blind cylindrical hole —
tool the circle prism r = 0.5, axis Z, footprint strictly inside,
z ∈ [−0.5, 1.5]: the floor at z = −0.5, the top rim at z = 1. The
derived counts (D5, the derivation visible): **V=12, E=17, F=8, H=1,
χ=2, genus 0**; volume abc − πr²(z_top − z_cap) = 8 − 0.375π.

*The formula, derived under the pinned conventions:* for a chain tool
of n entities: V = V_A+2n, E = E_A+3n, **F = F_A+n+1** (the walls + the
internal cap), **H = 1** (one pierced face) → **χ = χ_A**: the blind
hole is topologically flat — χ-preserving, the dent family (the same
class as the fillet's own χ-flatness, ADR-0033 D3's words) — where the
through hole is the handle family, χ = χ_A − 2. The circle case:
V = V_A+4, E = E_A+5, F = F_A+2, H = 1.

#### 2. LO SBORDO (the CROSSED) — the non-annular pierced face

*Today's refusal:* subtract.hpp:1140-1152 (pinned :952) — the tool's
profile pokes beyond the pierced face's boundary: "the non-annular
pierced face is honest G2 topology, not G1". The containment bands
(:1112-1137) sort the edges into `crossing` (margin > tol) and
`tangential` (|margin| ≤ tol), the canonical key ordering the triggers.

*The class, derived.* The door opens ONLY beyond the band: the
**crossing family** (margin > tol — the footprint exits transverse
through the pierced face's boundary plane) is admitted; the
**tangential family** (within the band) keeps today's refusal
byte-identical — that is LA PARETE's border (item 3) and the genuine
pinch; the discrimination is the band's own, zero new constants. The
admitted family's shape: the pierced face's region minus the footprint
becomes simply connected — **the face loses the corner, and its OUTER
loop is re-authored** around the notch (the walk's re-authoring); the
exit faces (where the notch leaves the target) shrink, their outer
loops re-authored; the tool's surviving walls re-window to the
footprint's clip; the tool's outside walls and caps vanish; the exit
edges (the target's face × the tool's wall — the plane×plane entry) and
the notch rims (the pierced face × the tool's wall at the pierced
quota) are born of the SSI grammar; the joint verticals re-time
(absorbed — the vocabulary row THIS unit adds, D2/D3).

*The guide case:* the box [−1,1]³ with the corner notch spilling over
the top — tool the square prism, footprint [0.5,1.5]×[0.5,1.5] at the
(+X,+Y) corner of the top face, z ∈ [−1.5, 1.5] through. The derived
counts (D5): **V=12, E=18, F=8, H=0, χ=2, genus 0**; volume
abc − (the corner column) = 8 − 0.5 = 7.5. The pierced face loses its
(1,1) corner — the face that loses the corner, the outer loop
re-authored; the right (+X) and back (+Y) faces shrink (absorbed);
left/front are kept; no operand face splits — the corner family is
deliberately the split-free variant of the class (the split family is
queued, D4, by name).

#### 3. LA PARETE-NEL-PIANO (the atrium of ADR-0037 D3.6) — the derived decision

*Today's refusal:* the TANGENT band, subtract.hpp:1153-1169 (pinned
:914) — the profile's edge within the band of the footprint's edge: the
tool's lateral wall lies IN the target's face plane, the zero-thickness
region contact.

*The decision, DERIVED — not deferred by fatigue:* **the parete stays
REGISTERED; it does not land in this unit.** The reason, in full:

1. The class's honest answer requires the **operand-face SPLIT**: the
   coplanar contact strip cuts the exit face into k pieces (the
   mid-strip family) or composes with the crossed corner (the shrink
   family) — either way one gesture THIS unit's classes never derive:
   the blind and the crossed-corner keep the 1:1 absorbed
   correspondence (one face in, one face out, the loop re-authored);
   the split breaks the assembly's own rule ("A's entities copied
   VERBATIM") — the operand's face is REPLACED by k faces.
2. The split's naming row — k names from one face's name — has no
   grammar today ("the naming of a re-trimmed outer loop does not
   exist" is one step short of it); its first consumer is the
   resolver's ambiguous class (ADR-0036 D3.2's own registration: "the
   row has MULTIPLE living descendants — the reference does NOT
   guess").
3. The coplanar wall disposition — "a wall is not a duplicate
   boundary; it IS the boundary" — is the region algebra's difference
   cell (ADR-0037's declared gift, available), but its consumer in the
   exact path is the parete itself, and **no guide case and no fillet
   class reaches the parete in this unit's scale**: the fillet's two
   classes (plane×plane, plane×cylinder ⊥ at the convex rim) do not
   touch a coplanar wall contact.
4. **The seat: the fillet's unit** — the next unit that extends the
   operand faces' walk re-authoring (the fillet's adjacent faces
   shrink, its end faces lose corners), where the split gesture, the
   wall disposition, and the crease merge (ADR-0037 D4's own queue:
   "the fillet's flank chains are the natural first") are priced by
   their consumer. There the TANGENT refusal is narrowed honestly (the
   margin-0 coplanar family admitted, the genuine pinch still refused);
   today it stands byte-identical, and the refusal IS the answer.

*One correction this derivation owes ADR-0037's record:* D3.6 pinned
the region algebra to "G2's unit when it builds the SSI-driven cap
insertion" — this unit. The derivation here: the cap insertion does
NOT consume the region algebra (no coplanar region contact in the
blind or crossed families — the SSI table's plane×cylinder entry
suffices); the gift's consumer is the parete, which stays registered.
**The gift travels untouched, registered with the parete to the
fillet's unit.** The derivation, not the expectation, is the authority.

#### 4. I VINCOLI DELLA CLASSE G1 — the additivity clause

The straight hole of always is NOT touched: the through family's
branches (the miss, the SAT touch, the swallow, the through assembly)
byte-identical; the G1 goldens (the through-hole's counts, the flush
cap, the miss, the oblique refusal) and the coplanar door's goldens
(P4/P8/P11 and the ratifiche P1/P2/P7/P10) do not move ONE BYTE. The
BLIND and CROSSED refusal pins (:952/:973) are the walls' own pins:
they BECOME the red tests of the new doors (D5) — the transformation
declared in the morsi's commit bodies, the red run cited there. The
families that stay refused keep their refusals byte-identical: the
TANGENT (the parete's border, pin :914 unmoved) and the
both-caps-inside family (the internal cavity — the BLIND message
narrowed honestly in m2 to name it, D4's queue). **The additivity
proof is a clause of THIS ADR:** the new branches seat inside the
existing tree at the pinned places (the crossing sort :1127-1131, the
z-band test :1173-1174), the admitted families are disjoint from the
kept refusals by the band's own margins, and the morsi's first check
is the full G1 suite green WITHOUT re-pinning — a count that moves is
a defect, not a golden (ADR-0035 D5's own words).

### D2. The algorithm — the SSI-guided cap insertion, the walk re-authorization, the tolerances, the output gate

**The cap insertion, guided by the SSI.** The seat: the z-band branch
(subtract.hpp:1172-1191). The walk table's blind extension authors, in
the tool's canonical walk order:

- the **internal cap face** — the tool's cap at end-of-course, carried
  with its plane (the same mathematical plane) and its provenance (the
  combined record, worst-of-operands, exact∧exact=exact), re-oriented:
  the region bits flipped, the loop re-authored as the rim arc pair
  split at the seam pair (ADR-0023's cap-loop form
  `[e_top1 fwd, e_top2 fwd]`) — the material now outside the profile;
- the **floor's rim** — the SSI consultation on the intra-tool pair
  (the tool's cap × the tool's wall, the same body twice —
  ssi.hpp:229's own form): the entry plane×cylinder ⊥ answers the
  circle at the cap quota with the p-curves in BOTH domains, and the
  walk emits the rim from the entry's both-domain answer — the curve
  at cap quota from the G2-u1 table, the insertion guided by her;
- the **wall's window** re-timed to [z_cap, z_top] (the existing
  re-timing machinery, the span clipped at the target's quota), the
  seam re-wired with the window;
- the **pierced face's inner loop** at the through side — the existing
  G1 machinery, unchanged.

**The walk re-authorization** (the crossed class, and the vocabulary
bill this unit pays). The walk tables (the NUMBERING tables of
extrude.hpp/subtract.hpp, the naming role tables, the descent)
gain the rows:

1. **The joints' verticals** — the morse registers' own gap ("the
   joints' verticals have no vocabulary row yet"): the NUMBERING
   already carries them (extrude.hpp "2n+i = the vertical at J_i";
   subtract.hpp "E_A+2n+i = the vertical at J_i"); the NAMING row is
   added here: **`joint(i)`** — the vertical at the chain's joint i,
   chain-owned (the multi-chain group prefix exactly as `lat`), the
   genealogy the authored root's (the sketch's vertex i — the root
   reference's own form). First instances: the crossed guide case's
   absorbed `ext#3:joint(3)` (re-timed to the box's span); the
   extrude's own bodies answer it on every prism.
2. **The re-authored outer loop** — the walk's row for the pierced
   face's re-trimmed outer loop (the numbering: the edge sequence in
   the canonical walk order — the kept originals shortened, the born
   isect rims, the born exit edges; the naming: the born edges carry
   the isect grammar, the absorbed keep the operand's own names; the
   descent: the face absorbed with the loop change — the morse-4
   register's own mark, "the loop changes where it trims").
3. **The internal cap's rows** — the numbering row (the walk table's
   face rows gain the cap: the circle case F_A+1); NO new face role:
   the cap is the tool's own, absorbed — the subtract's face table
   stays EMPTY (the morse-4 register's own honesty), the stamp
   inherited at the assembly site.

**The tolerances — zero new constants; ε_line DECIDED.** Every band
this unit reads is already typed and declared: the placement margins
and z-bands read the operation's Layer-3 `tol` (the exact_subtract
discipline, each use documented at its site); the crossing/tangential
sort reads the same `tol` — the door's discrimination is the band's
own. **The ε_line question (ADR-0037 D2's sentinel — "whether ε_line
should SURFACE in the result's Layer-2 record rather than living only
in this derivation") is decided here: IT STAYS A DECLARED DERIVATION.**
The reason, written: the Layer-2 field is the bound on the STORED
geometry versus the NOMINAL geometry (ADR-0031 D3 Layer 2's own
definition) — and for the only class the exact table serves (the
provenance gate refuses approximated operands, ssi.hpp:87-89), the
operands' stored IS the nominal (factory-exact), so the transversal
line's stored position IS the nominal line, and the pairing rule's
exact/0 is TRUE — "the pairing IS the contract" (ssi.hpp's own words).
Surfacing ε_line there would write a false number: a bound on a
deviation that is zero. ε_line is the SENSITIVITY of the answer to the
operands' trust radius (the conditioning, 1/sin θ — the sentinel's own
derivation, monotone, named) — a different question, which no consumer
asks today: the report, the mass and the resolver read the record as
it is. The first consumers that will ask it are option (ii)'s
subdivision citizens and the fillet's flank conditioning at the
near-band rim; THAT unit extends the record's vocabulary with the
consumer named. **Consequence: ADR-0032 is NOT amended** (the surgery
list's conditional not fired); the sentinel stays where it lives
(ADR-0037 D2, the site's comment), cited from here.

**The coherence with the output gate** (the honest-solid classes):
every spigolo exactly 2 opposite co-edges (the radial structure derived
per class in D5 — the rim arcs shared floor/window and inner/window,
the seam's periodic identification, the exit edges shared
face/wall); χ even (both classes: χ = 2); every loop closed (the
re-authored walks close on themselves — the derivation shows the
sequences); the output gate (boolean.cpp:1462-1569's family) accepts
the bodies, and the χ identity is verified against it — the derivation
VISIBLE, never just the number (D5).

### D3. The names — the vocabulary, the tavola per class, the fillet as the declared consumer

**The internal cap: ABSORBED — one correction to the order's sketch,
on the record.** The order's D1/D3 sketched "la CAP INTERNA nasce …
born, il suo nome dalla grammatica SSI". The derivation says
**absorbed**, and the derivation is the authority (ADR-0033's
acceptance record, the house's own sentence): the internal cap is the
tool's own cap face at the end of its course, surviving 1:1 into the
result, MODIFIED — the loop re-authored (the rim arc pair at the cap
quota, the seam split), the region bits flipped, the stamp inherited,
the record combined — **exactly the wall's own disposition**
(ADR-0036 D1's table: "the hole's cylindrical wall ext#3:lat(0)
absorbed (re-framed to the window, region-on-right)"; the morse-4
register: "the wall is the tool's own lateral ABSORBED, whose stamp is
INHERITED"). A born cap would need a NEW face role for the subtract
(whose face table is honestly EMPTY) and would leave the tool's cap
row unexplained — one face in, one face out is the absorbed mark; two
tool faces surviving 1:1 (the wall and the cap) cannot take different
relations without a reason, and none exists. What IS born of the SSI
grammar is the cap's RIM — the intra-tool pair's curve (below) — and
that is where "il suo nome dalla grammatica SSI" honestly lands: the
fillet's reference reads the rim, not the face.

**The names, per class** (the grammar's rows this unit adds, the
consumers named):

- **The blind case's born curves:** the top rim —
  `sub#4:isect[box#1:top × ext#3:lat(0)]@hi` (the G1 form, unchanged);
  the floor's rim — **`sub#4:isect[ext#3:cap(lo) × ext#3:lat(0)]`** —
  the grammar's FIRST intra-body row (the same body twice; the level
  suffix ABSENT: the cap plane IS the level — the overlay's cross-edge
  elision, ADR-0037 D6's own form); the wall's seam —
  `sub#4:seam[ext#3:lat(0)]` (the G1 form, unchanged).
- **The crossed case's born curves:** the notch rims —
  `sub#4:isect[box#1:top × ext#3:lat(0)]@hi`,
  `sub#4:isect[box#1:top × ext#3:lat(3)]@hi` (and the @lo mirror on
  box#1:bottom); the exit verticals —
  `sub#4:isect[box#1:right × ext#3:lat(0)]` and
  `sub#4:isect[box#1:back × ext#3:lat(3)]` (the level absent: the
  pair's planes ARE the line — the plane×plane entry, landed Phase D).
- **The absorbed rows:** the pierced/re-authored faces keep the
  factory's names (box#1:top, box#1:right, …); the walls keep the
  extrude's (ext#3:lat(0), ext#3:lat(3)); the internal cap keeps
  ext#3:cap(lo); the joint vertical keeps ext#3:joint(3) — the new
  row's first instance.

**LA TAVOLA DELLE DISCESE, per class, with the counts** (the relation
vocabulary closed: kept/absorbed/split/merged/vanished; the closure
property hard):

- *The blind guide case (47 rows — the same count as G1's own table,
  derived):* faces 9 (out 8 = 5 kept + 3 absorbed [the pierced top,
  the wall, the internal cap]; vanished 1 [the tool's far cap]);
  edges 22 (out 17 = 12 kept + 5 born [the floor rim pair, the top rim
  pair, the seam]; vanished 5 [the tool's rims and seam, re-authored
  with the window]); vertices 16 (out 12 = 8 kept + 4 born; vanished
  4 [the tool's split pair]). The blind trades G1's second pierced
  absorption for the cap's: out+vanished is 9/22/16 both ways.
- *The crossed guide case (64 rows, derived):* faces 12 (out 8 = 2
  kept [left, front] + 6 absorbed [top, bottom, right, back, the two
  surviving walls]; vanished 4 [the two outside walls, the two caps]);
  edges 30 (out 18 = 7 kept + 5 absorbed [the four shortened box
  horizontals, the joint vertical] + 6 born [the four notch rims, the
  two exit verticals]; vanished 12 [the box's dead corner column edge,
  the tool's eight chain curves, the tool's three outside verticals]);
  vertices 22 (out 12 = 6 kept + 6 born; vanished 10 [the box's two
  dead corners, the tool's eight joint instances]).

**IL FILLET COME CONSUMATORE DICHIARATO** — the contract lives before
the consumer (ADR-0036's method, replicated): the Feature→Fillet will
read, by name: the blind hole's floor rim
`sub#4:isect[ext#3:cap(lo) × ext#3:lat(0)]` (the plane×cylinder class
— the rolling ball at the floor's convex rim), and the notch's rims
`sub#4:isect[box#1:top × ext#3:lat(0)]@hi` (the plane×plane class —
the cylindrical flank). Those strings are this ADR's contract with the
fillet, pinned here before the fillet exists, so the fillet cannot
bend them.

### D4. The v1 scope and the queue — MAI "tutte", each by name with its reason

**IN v1 (the two morsi's reach — what the guide cases and the fillet
REALLY touch):**

1. **The blind family** (exactly one cap strictly inside, the other
   side through; the tool in the authored class; the footprint strictly
   inside the pierced face): the internal cap carried, the rim typed by
   the intra-tool SSI consultation, the counts 12/17/8/1 χ=2.
2. **The crossed corner family** (the footprint exiting transverse
   beyond the band, at the pierced face's corner — the split-free
   variant): the outer loop re-authored, the exit edges and rims born,
   the counts 12/18/8/0 χ=2.
3. **The constraints**: the G1 class and the door's goldens unmoved
   (D1.4); the tangential and both-inside families still refused.

**OUT of v1, the queue — by name, never footnoted:**

- **LA PARETE-NEL-PIANO** — registered to the fillet's unit (D1.3's
  derived decision: the split gesture, the naming row, the region
  algebra's difference cell; no consumer in this scale; the TANGENT
  refusal byte-identical meanwhile).
- **The split family** (the crossed mid-edge and the parete strip: one
  operand face → k faces) — rides the parete; the first consumer is
  the resolver's ambiguous class (ADR-0036 D3.2's own registration).
- **The blind-crossed composite** (the blind notch: the floor
  non-convex, an L-shaped internal cap; the crossed family with the
  blind z-span) — the classes compose; the composite's walk
  re-authoring is its own derivation, priced by its first real case.
- **The fully-internal cavity** (both caps inside: the two-shell
  body, χ = 2·2) — the multi-shell honesty (the report's
  single-shell-only derivation rule, ADR-0027 D6's own amendment);
  the family keeps its refusal (the message narrowed in m2 to name
  it).
- **The circular crossed** (the circle tool crossing: the rim arcs on
  the re-authored outer loop — the Arc2 CCW convention at the reflex
  parts) — the arcs' convention derivation, priced by its consumer.
- **The sphere booleans** (the old act one's promise: sphere×sphere,
  sphere±prism, the truncation family, the trimmed-sphere mass family,
  the two-holes body on a pierced target) — re-queued as their own
  unit; the harvest's counts travel there (RACCOLTO's list), to be
  re-derived on the redeemed foundations, never copied.
- **The ε_line sensitivity row** — decided (D2): stays a derivation;
  re-opened by its consumer (option (ii)'s citizens, the fillet's
  flank conditioning).
- **The crease merge** — ADR-0037 D4's own queue item; untouched here
  (the fillet's flank chains the natural first).
- **The unequal-radii parallel pair** (the SSI table's own pointer:
  the transverse case typable, awaiting its consuming unit) and
  **ADR-0036 D4's own queue** (the vertex names, the polygon path's
  tavola) — untouched here, by their owners.

### D5. The golden pre-pinnings — the derived counts with the derivation visible, and the naive counterproof

**The blind guide case, derived step by step** (box a=b=c=2; tool the
circle prism r=0.5, footprint centered strictly inside, z ∈ [−0.5,
1.5]; the floor quota z_cap = −0.5, the top quota z_top = 1):

- *Vertices:* the box's 8 (kept) + the walk's split pairs: S/A at the
  floor quota (2) and S/A at the top quota (2) — the rim circles split
  at the seam pair per ADR-0023 (the periodic face's own convention):
  **V = 8+4 = 12**.
- *Edges:* the box's 12 (kept) + the walk's: the floor rim's half-arc
  pair (2), the top rim's half-arc pair (2), the seam vertical
  S_cap→S_top (1) — the same five-kind as G1's circle case, the quotas
  re-based: **E = 12+5 = 17**.
- *Faces:* the box's 6 (the top absorbed + the other five kept) + the
  wall (absorbed, the window re-timed to [−0.5, 1]) + the internal
  cap (absorbed, re-oriented): **F = 6+2 = 8**.
- *H:* the top face's inner loop (the rim at z_top): **H = 1**; the
  floor's rim is its outer loop (no inner loop).
- *The χ identity, visible:* χ = 12−17+8−1 = **2**; Δχ = +4−5+2−1 =
  0 — **χ = χ_A: the dent family** (the through hole's Δχ = −2, the
  handle family). Genus 0 (χ = 2−2G, single shell).
- *The output gate:* every edge 2 opposite co-edges — the floor rim's
  arcs shared between the floor's outer loop and the wall's window
  bottom; the top rim's arcs shared between the top face's inner loop
  and the wall's window top; the seam carried twice by the wall's
  single loop (the periodic identification, the XOR satisfied); the
  box's edges verbatim. χ even; the loops closed (the floor's loop
  `[arc₁ fwd, arc₂ fwd]`; the wall's window walk region-on-right; the
  top face's inner loop CCW in its domain, hole=true).
- *The volume, the classical form is the test:* abc − πr²(z_top −
  z_cap) = 8 − 0.375π ≈ 6.8219; all_exact true; the centroid on the
  axis by symmetry.

**The crossed guide case, derived step by step** (box [−1,1]³; tool
the square prism, footprint [0.5,1.5]×[0.5,1.5], z ∈ [−1.5, 1.5]
through; the removed corner column [0.5,1]×[0.5,1]×[−1,1]):

- *Vertices:* the box's 8 → 6 (the (1,1) column's two corners die —
  no face contains them: the top/bottom faces lose the corner, the
  right/back faces shrink past them); + the born six: (1,0.5,±1) and
  (0.5,1,±1) (the exit verticals' ends) and (0.5,0.5,±1) (the inner
  corner of the L): **V = 12**.
- *Edges:* the box's 12 → 11 (the (1,1) vertical dies; the +X and +Y
  horizontals at both quotas are absorbed — re-spanned; the rest kept)
  + the born: the notch rims (two per pierced quota: the y=0.5 rim and
  the x=0.5 rim at z=±1 — four), the exit verticals (1,0.5) and
  (0.5,1) — two; + the absorbed joint vertical at (0.5,0.5) (the
  tool's joint(3), re-timed to [−1,1]): **E = 11+6+1 = 18**. The
  co-edge incidence check: 36 co-edges over the eight faces' loops /
  2 = 18 — every edge exactly two opposite co-edges, the count closed.
- *Faces:* the box's 6 (top/bottom/right/back absorbed — the outer
  loops re-authored; left/front kept) + the two surviving walls
  (lat(0) the y=0.5 wall, re-windowed to x∈[0.5,1]; lat(3) the x=0.5
  wall, re-windowed to y∈[0.5,1]): **F = 8**. The tool's lat(1)/lat(2)
  (the x=1.5, y=1.5 walls) and both caps vanish (cut away outside).
- *H:* **0** — the pierced faces are L-shaped, simply connected; the
  re-authored outer loop of the top face, walked in full: (−1,−1)→
  (1,−1) [kept] → (1,0.5) [absorbed, re-spanned] → (0.5,0.5) [born:
  isect[box#1:top × ext#3:lat(0)]@hi] → (0.5,1) [born:
  isect[box#1:top × ext#3:lat(3)]@hi] → (−1,1) [absorbed, re-spanned]
  → (−1,−1) [kept] — six edges, closed, the loop reads CCW in the
  face's domain (the material-on-left authoring's guarantee).
- *The χ identity, visible:* χ = 12−18+8−0 = **2**; Δχ = +4−6+2−0 =
  0 — the dent family again; genus 0, single shell.
- *The volume:* abc − (the corner column) = 8 − 0.5 = **7.5**;
  all_exact true.

**LA CONTROPROVA NAIVE — the refusals become the red tests.** The new
goldens (the counts above, the volumes, the name digests, the tavole's
closure, the determinism digests — N=10 rebuilds bit-identical, on
BOTH compilers) are written from THIS section BEFORE the code; run on
today's tree they fail AT THE REFUSAL — the BLIND at :973's family,
the CROSSED at :952's — the red run cited in the morsi's commit
bodies. The wall becoming a door is the discrimination: the door is
NEW — the counts, the names, the descent rows and the volumes are
asserted, never the refusal's message repainted. The pins :952/:973
retire with their families' admission, the transformation declared;
**the G1 goldens and the door's goldens do not move ONE BYTE** — the
morsi's first check is the existing suite green without re-pinning
(D1.4's additivity clause).

### D6. The consequences — the register surgery, the morsi 2+, the estimates

**The register surgery — declared; applied in the same act as
acceptance** (the ADR-0035 D6 protocol):

- **ADR-0033 — amended in place, dated, pointing here**: D5.2's
  unit-2 seat is re-derived honestly — the subtract's registered
  classes (ADR-0037 D4's own pointers) land first (this unit: the
  blind and the crossed; the parete registered to the fillet's unit
  by D1.3's derivation); the sphere booleans of the old act one —
  D5.2's original wording — are re-queued by name with the harvest's
  derivations traveling to their consumer. The promise is ratified
  with its payment schedule named, never dropped.
- **ADR-0036 — amended in place, dated, pointing here**: the morse
  registers' gap — "the joints' verticals have no vocabulary row yet"
  — is paid: the `joint(i)` row lands (this ADR's D2/D3), the "yet"
  redeemed. The mothers' own words stand; the queue items that remain
  (the vertex names, the polygon path's tavola) stay queued by their
  owners.
- **ADR-0032 — NOT amended**: D2 decided ε_line stays a declared
  derivation (the reason written in D2); the surgery list's
  conditional ("se D2 lo decide") does not fire.
- **ADR-0037 — NOT amended**: D3.6's gift travels untouched,
  registered with the parete to the fillet's unit (D1.3's correction,
  on the record here); the G2 boundary stands.

**The morsi 2+** (each lands standalone with the full suite green,
the order by scale — the blind first, the crossed after; STARTING
ONLY on the maintainer's review of THIS document):

1. **m2 — the blind** (IL FORO CIECO): the z-band branch's extension,
   the walk table's blind rows, the internal cap carried, the
   intra-tool SSI consultation at the cap quota, the `joint`/
   outer-loop vocabulary rows' first landing, the red-first goldens
   (12/17/8/1 χ=2; 8−0.375π), the tavola (47 rows), the bindings
   reachable from Python, the report's mass.
2. **m3 — the crossed** (LO SBORDO): the containment sort's admission
   beyond the band, the outer loops re-authored, the exit edges and
   rims born, the joint vertical re-timed, the red-first goldens
   (12/18/8/0 χ=2; 7.5), the tavola (64 rows), the bindings, the
   report.

**LE STIME, in the register** (the H2 checkpoint's lesson applied:
measured granularities, never promises — the stick is the H2
register's own dates: 2026-10-30 19:40 → 11-02 12:14, four units and
the phase checkpoint in ≈ 2.7 days): **m2 (the blind) ≈ 1 day; m3
(the crossed) ≈ 1–2 days** — the crossed is the unit's heavier walk
re-authoring (two outer loops, two faces shrunk, six born curves).
Estimates, labeled as such; the tail (the queue's items) carries NO
date — the no-promises list, verbatim from the mothers: no date, no
sprint, no percentage on anything past m3.

**The no-promises list:** no code moves in m1 (this delivery is this
document alone); no SSI entry is added (the table stands; the
intra-tool consultation uses the landed plane×cylinder ⊥ and
plane×plane entries); no format bump, no CI change, no new
dependencies; the polygon path and the coplanar door are untouched;
the sphere booleans are re-queued, never scheduled here; the parete
carries no date.

## Consequences

- The exact subtract's registered walls become doors where the
  derivation admits them, and only there: the blind hole and the
  corner notch answer with true faces, exact counts, and names the
  fillet will read — each door's goldens red-first, the existing
  goldens unmoved, the kept refusals byte-identical.
- The walk tables' vocabulary gap closes where the classes forced it:
  the joints' verticals and the re-authored outer loop gain their
  rows — the register's "yet" redeemed in the act this ADR is
  accepted.
- The old branch's experiment is cashed in: its counts travel to the
  sphere booleans' re-derivation, its lesson (the code fought the
  record; the record won) is dated in the register, and not one of
  its bytes enters the redeemed kernel.
- The ε_line question is answered with a reason, not a shrug: the
  record stays honest (exact/0 where the pairing is the contract),
  the sensitivity stays a named derivation, and the first consumer
  that needs it will extend the vocabulary with its own name.

## References

House records: ADR-0033 (the G2 boundary and ladder re-ordered here,
D5.2 amended); ADR-0031 (D4.2 the boolean guarantees, D5 the scale
method, the checkpoint's measuring lesson); ADR-0035 (the typed
tolerance worlds — every band this unit reads; D5 the golden
protocol); ADR-0036 (the naming grammar D3 extends, the descent
vocabulary, the morse registers' gap paid here); ADR-0037 (D4's queue
ratified, D2's sentinel decided, D3.6's gift forwarded with the
correction); ADR-0023 (the seam conventions the derived counts apply);
ADR-0032 (the Layer-2 contract re-read, not amended); ADR-0034 (the
arithmetic boundary the old branch fought — the RACCOLTO's lesson);
ADR-0014 (determinism — the digests' bar); the G2-u1 table (ssi.hpp/
ssi.cpp — the intra-tool consultation's surface); the G1 code
(subtract.hpp:84-88 the pinned formulas, :1046-1191 the decision tree
cited branch by branch, :113-133 the walk tables); naming.hpp (the
role tables D3's rows join); the git register (3615453, a62f564,
52915c0 — the RACCOLTO's own citations).

The literature the derivations lean on, as ever: Mäntylä 1988 (the
Euler operators the χ arithmetic stands on); Hoffmann 1989 (the
blending chapters — the vertex-blend tail this ADR keeps refusing by
name); and the classical forms that are the tests, never the
implementations: the spherical cap πh²(3R−h)/3 (the harvest's family),
the cylinder patch πr²h (the blind hole's volume).
