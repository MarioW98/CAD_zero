# ADR-0002: Dual-representation kernel with native SDF

## Status
Accepted (2025-10-07). Supersedes the original proposal's
"Shape with payload {sdf?, brep?}" approach.

## Context
The original proposal suggested a `Shape` whose payload was:
```
payload:
  sdf:  optional<SDFNode>
  brep: optional<BRepBody>
```
with the rule "if both are non-null, it's a hybrid". This is ambiguous
and forces every consumer to handle 4 combinations instead of 3, plus
makes `std::visit` dispatch impossible.

## Decision
1. **SDF is a first-class native representation**, on equal footing with
   B-Rep. It is not a "lattice/organic" fallback; it is one of two
   primary representations.

2. **`Shape.payload` is `std::variant<SDFBody, BRepBody, HybridBody>`**.
   The three states are mutually exclusive at the type level. The
   `HybridBody` is a first-class type that *owns* both an SDF and a
   B-Rep, plus a bridge tolerance.

3. **Operations are dispatched by `std::visit`**, not by `if-else`
   chains. The visitor pattern is exposed as `Shape::accept(Visitor&)`.

4. **CSG operations exist natively in both domains**: SDF has its own
   union/subtract/intersect/smooth_min, B-Rep has its own. The system
   picks the implementation based on operand types, never silently
   converting one representation into the other.

5. **Conversion is explicit and tracked**: a "Convert to SDF" or
   "Convert to B-Rep" feature is inserted into the feature tree, and
   its declared tolerance accumulates into the downstream shapes'
   `ToleranceAccumulator`.

## Consequences
- `Shape` is move-only (B-Rep bodies own substantial state).
- The visitor base class (`ShapeVisitor`) is the integration point for
  plugins that need virtual dispatch.
- Python bindings expose `representation()` returning a `Representation`
  enum; accessors `as_sdf()`, `as_brep()`, `as_hybrid()` return
  nullable pointers.

## References
- Siemens NX "Convergent Modeling" whitepaper
- nTopology / nTop implicit modeling documentation
