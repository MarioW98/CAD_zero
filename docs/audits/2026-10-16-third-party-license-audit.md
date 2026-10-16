# Audit licenze di terze parti — 2026-10-16

## Scope e metodo

Inventario di **ogni dipendenza diretta** del progetto (C++ e Python, incluse
quelle test-only/build-only), con licenza, versione, uso e un flag esplicito
di compatibilità con **derivati parzialmente closed-source** (lo scenario
"core aperto + parti chiuse" dichiarato in ADR-0013). Le licenze C++ sono
state verificate **leggendo i file LICENSE dei sorgenti effettivamente
fetchati** in `build/_deps/` (non dalla memoria): fmt, spdlog, doctest,
xsimd, nanobind. Completata l'inventario, è stata eseguita una verifica
anti-codice-copiato (grep per header copyright di terzi) su tutto il codice
del repo.

**Punto fermo (richiesta dell'utente): la scelta della licenza del progetto
non è stata toccata.** Il file `LICENSE` esiste già a root ed è Apache-2.0
completo (sezioni 1–9, testo canonico senza appendice); la raccomandazione
che segue è confermativa, non prescrittiva.

## Stato attuale della licenza del progetto

- `LICENSE` (root): **Apache-2.0**, testo completo §1–9, 201 righe.
- `pyproject.toml`: `license = { text = "Apache-2.0" }` + classifier
  OSI "Apache Software License" — coerente.
- `README.md`: badge shields.io Apache-2.0 + sezione License finale —
  coerente (il riferimento a "Boost" era stale, vedi Findings; corretto
  in questa commit).
- `ADR-0013`: policy di compatibilità accettata — GPL-3+ vietato,
  CGAL differita, NOTICE richiesto alla prima release pubblica.
- About dell'app: dichiarava "License: Apache-2.0" **con** testo stale
  "Phase C+" — reso coerente con lo stato reale in questa commit
  (la dichiarazione era già legittima: il LICENSE a root la concedeva).

## Dipendenze C++ (fetch via CPM.cmake, pin per tag)

| Dipendenza | Versione (pin) | Licenza (verificata) | Uso in CAD_0 | Derivati closed? |
|---|---|---|---|---|
| fmt | 10.2.1 | MIT-style "{fmt} license" (Zverovich, con eccezione attribuzione) | formattazione/logging, linkata nelle librerie | ✅ permissiva |
| spdlog | v1.14.1 | MIT (Melman) | logging async | ✅ permissiva |
| doctest | v2.4.12 | MIT (Kirilov) | framework unit test (**test-only**) | ✅ permissiva |
| xsimd | 12.1.1 | BSD-3-Clause (QuantStack/Mabille et al.) | SIMD in `core/sdf/src/evaluate.cpp` (`<xsimd/xsimd.hpp>`) | ✅ permissiva |
| nanobind | v2.2.0 | BSD-3-Clause (Jakob) | bindings Python (**build+link dell'estensione**) | ✅ permissiva |
| oneTBB | v2022.0.0 | Apache-2.0 | thread pool — **opzionale, OFF di default** (`CAD_0_USE_TBB`) | ✅ stessa licenza del progetto |

Nessuna dipendenza C++ è copyleft: il **kernel C++ linkato a closed-source
resta privo di obbligazioni virali**.

## Codice vendored nel repo

| Voce | Licenza | Note | Derivati closed? |
|---|---|---|---|
| `third_party/robust_predicates/predicates.c.h` | Public Domain (Shewchuk) | il file in-tree è uno **stub Phase A scritto dal progetto** (non-adaptive fallback); il `robust.c` reale da scaricare è public domain per dichiarazione dell'autore (pagina CMU) | ✅ |
| `cmake/CPM.cmake` | MIT (upstream CPM 0.40.2) | bootstrap minimale che scarica CPM al primo configure (build-time only) | ✅ |

## Dipendenze Python (pyproject.toml — floor, non pin)

| Dipendenza | Floor | Licenza | Uso in CAD_0 | Derivati closed? |
|---|---|---|---|---|
| numpy | >=1.24 | BSD-3-Clause | ndarray view delle mesh (bindings + UI) | ✅ permissiva |
| matplotlib | >=3.5 | Matplotlib License (BSD-style, permissiva) | viewer headless (`bindings/python/CAD_0/viewer.py`) | ✅ permissiva |
| PySide6 | >=6.6.0 ([ui]) | **LGPL-3.0** | l'intera app desktop | ⚠️ vedi sotto |
| PyOpenGL | >=3.1.0 ([ui]) | MIT | chiamate GL del viewport | ✅ permissiva |
| typer | >=0.12 ([cli]) | MIT | CLI (`CAD_0.cli:app`) | ✅ permissiva |
| rich | >=13.0 ([cli]) | MIT | output CLI | ✅ permissiva |
| pytest | >=8.0 ([dev]) | MIT | test | ✅ (non distribuita) |
| pytest-xdist | >=3.5 ([dev]) | MIT | test paralleli | ✅ (non distribuita) |
| mypy | >=1.10 ([dev]) | MIT | type check | ✅ (non distribuita) |
| ruff | >=0.5 ([dev]) | MIT | lint | ✅ (non distribuita) |
| black | >=24.0 ([dev]) | MIT | format | ✅ (non distribuita) |
| scikit-build-core | >=0.10.0 ([build]) | Apache-2.0 | backend wheel | ✅ (build-time) |
| nanobind | >=2.2.0 ([build]) | BSD-3-Clause | backend bindings | ✅ (build-time) |

Le transitive (es. shiboken6, anch'essa LGPL) non sono inventariate:
l'audit copre le dirette, come richiesto.

### PySide6 (LGPL-3.0) — l'unico copyleft dell'albero

- **Compatibilità con parti chiuse: condizionata ma praticabile.** LGPL-3.0
  consente uso in prodotti closed **a patto che**: (1) il collegamento sia
  dinamico — con PySide6 avviene all'import Python, caso standard; (2)
  l'utente finale possa **sostituire** la libreria (con wheel pip è la
  condizione naturale); (3) licenza e copyright notice di PySide6/Qt
  accompagnino la distribuzione. Vietato lo static linking di Qt.
- **Architettura già favorevole**: il copyleft è confinato nello strato
  UI ([ui] extra). Il kernel C++ e i bindings non toccano Qt: un derivato
  closed che riusa solo il core non ha obbligazioni LGPL; un derivato
  closed che include l'app deve solo rispettare i tre punti sopra.
- ADR-0013 vieta già GPL-3+ e impone dynamic linking per LGPL: politica
  coerente con questo quadro.

## Verifica anti-codice-copiato

Grep su `core/ app/ bindings/ examples/ tests/` per header copyright/SPDX/
GPL di terzi: **un solo hit**, un commento in
`bindings/python/tests/test_import_roundtrip.py` che documenta il simbolo
linker `GNU_UNIQUE` (documentazione tecnica, non codice terzo). Nessun
file sorgente del progetto contiene codice copiato con licenza restrittiva.

Citazioni algoritmiche presenti (tutte a **letteratura**, non codice):
Greiner-Hormann (1998), Sutherland-Hodgman, MC33 (Chernyaev/Lewiner,
Hamann 1991), smooth-min polinomiale (Quilez). Le formule/algoritmi non
sono copyrightabili; le implementazioni in-repo sono originali con
derivazioni in-file (es. la chain-rule di `SmoothMinSDF` in
`operators.hpp`). Le tabelle di marching cubes sono **calcolate** da
`kEdgeCorners` (nessuna copia della classica tabella 256-voci alla
Bourke; la tabella morta `kEdgeAnchors` è stata rimossa in precedenza).

## Findings

1. **`LICENSE` esiste ed è Apache-2.0 completo** → nessuna decisione
   licenza pendente; l'About che dichiara Apache-2.0 era legittimo (il
   testo stale "Phase C+" fu corretto in questa commit).
2. **ADR-0013 è superato**: copre solo il lato C++, ignora le dipendenze
   Python (e quindi PySide6/LGPL — l'unico copyleft reale), cita
   CGAL/OpenVDB mai adottate, e il **NOTICE** che essa stessa dichiara
   "richiesto per la prima release pubblica" non esiste ancora. Da
   aggiornare prima di una release pubblica.
3. **README §License citava "Boost"**: nessuna dipendenza Boost esiste
   (residuo storico). Corretto in questa commit; aggiunto il riferimento
   LGPL di PySide6 per lo [ui] extra.
4. **Wheel metadata**: `license = { text = "Apache-2.0" }` funziona, ma
   per una release pubblica valutare PEP 639 (`license = "Apache-2.0"` +
   `license-files`) e il file NOTICE (anche per le note di attribuzione
   Apache-2.0 §4).

## Raccomandazione (confermativa — la licenza esiste già)

Per lo scenario "core aperto + parti chiuse":

- **Apache-2.0 (attuale)**: la scelta corretta. Consente esplicitamente
  derivati closed (§2), ha **grant di brevetto esplicito** (§3 — MIT non
  lo ha), e la convenzione NOTICE gestisce l'attribuzione. Costo: testo
  lungo e formalità NOTICE in distribuzione.
- **MIT**: più corta e semplice, nessun grant di brevetto esplicito
  (resta la protezione implicita, più debole). Cambiare ora non porterebbe
  vantaggi concreti e richiederebbe il consenso di tutti i contributor.
- **Nessuna licenza**: "all rights reserved" di fatto — bloccherebbe ogni
  derivato legale. Da escludere.

**Conclusione: mantenere Apache-2.0.** Nessuna azione richiesta; la scelta
finale resta dell'utente (questo audit non decide, documenta).
