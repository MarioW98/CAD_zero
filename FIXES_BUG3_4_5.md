# CAD_0 — Correzioni Bug 3 / 4 / 5

Basato su: repo `MarioW98/CAD_zero`, commit `ac75bc4` ("majr fixes", che
conteneva già i fix di Bug 1 e Bug 2 consegnati in precedenza).

---

## Bug 3 — Doppia UI divergente (copia morta)  — RISOLTO

**Problema.** Il progetto conteneva due copie della UI PySide6:

| Copia | Stato | Note |
|---|---|---|
| `app/src/CAD_0/ui/` | **usata davvero** | il build la copia in `build/python/CAD_0/ui/` |
| `bindings/python/CAD_0/ui/` | **codice morto** | versione pre-10/09: viewport senza PyOpenGL, export con la vecchia condizione invertita |

Il build copiava prima la copia di `bindings/` e poi la sovrascriveva con
quella di `app/src/` — quindi la copia in `bindings/` non aveva alcun effetto,
ma modificarla dava l'illusione di aver sistemato le cose. Divergenza: 260
righe in `main_window.py`, 391 in `viewport_widget.py`.

**Correzione.**
- Eliminata `bindings/python/CAD_0/ui/` (6 file).
- Aggiornato il commento in `bindings/python/CMakeLists.txt`: la sorgente
  unica della UI è `app/src/CAD_0/ui/`.
- Verificato: il `build/python/` risultante è identico al byte.

---

## Bug 4 — Sicurezza lifetime nei binding nanobind  — RISOLTO

**Problema reale (rischio use-after-free).** In
`bindings/python/src/mesh_bindings.cpp` le proprietà `positions` / `normals` /
`indices` di `TriangleMesh` restituivano ndarray numpy costruiti con owner
**vuoto** (`nb::handle()`): gli array puntavano la memoria C++ dei vettori
senza trattenere in vita il mesh. Cioè:

```python
arr = scene.node_at(0).cached_mesh.positions
del scene            # mesh C++ liberato -> arr ora punta memoria liberata
```

**Correzione.** Owner esplicito `nb::find(&mesh)` (pattern canonico nanobind):
l'ndarray trattiene il wrapper Python del mesh → la memoria C++ vive finché
vive l'array. Dimostrato dai test: refcount del wrapper +1 dopo l'accesso alla
proprietà; dati intatti dopo `del` + `gc.collect()`.

**Pulizia policy fuorvianti.**
- `Scene.next_undo_description` / `next_redo_description`: rimosso
  `rv_policy::reference` (il ritorno `const std::string&` viene copiato in un
  `str` immutabile dal caster — la policy non legava alcun lifetime ed era
  ingannevole).
- `weld_vertices`: rimosso `rv_policy::reference_internal` (ritorna
  `std::size_t`, la policy non significava nulla).

**Verdetto sul report "nanobind: leaked instances/functions" a shutdown.**
Sonda memoria su 300 iterazioni complete (scena+mesh+ndarray+CommandStack):
RSS stabile a 33.8 MB, crescita **0.00 MB**. Il report scompare del tutto con
cleanup esplicito prima dell'uscita → artefatto dell'ordine di distruzione
dell'interprete, **non** un leak di heap.

---

## Bug 5 — Dipendenze e percorsi  — RISOLTO

**PyOpenGL mancante.** `pyproject.toml` dichiarava `ui = ["PySide6>=6.6.0"]`,
ma `viewport_widget.py` importa `OpenGL.GL` in modo diretto: un
`pip install CAD_0[ui]` lasciava l'app non avviabile.

```toml
ui = ["PySide6>=6.6.0", "PyOpenGL>=3.1.0"]
```

**Percorsi hardcoded.** `scripts/start_codespaces.sh` fissava 4 percorsi
assoluti `/home/z/my-project/...` (layout della sandbox originale). Ora:

- `PROJECT_DIR` derivato dalla posizione dello script (funziona da qualsiasi
  checkout);
- `.libs/` e `vnc_extract/` risolti come *sibling* del progetto (layout
  bundle: `<bundle>/CAD_0/` + `<bundle>/.libs/` + `<bundle>/vnc_extract/`);
  se assenti (repo pura) scattano i fallback apt/sudo, come su Codespaces;
- override opzionali: `CAD_0_PROJECT_DIR`, `CAD_0_LIBS_DIR`,
  `CAD_0_VNC_EXTRACT_DIR`.

**Nota sandbox:** `libQt6Gui.so.6` linka `libEGL.so.1` — se il sistema non la
fornisce, serve lo staging `.libs/` (come nel tuo bundle originale). Nella
repo non è inclusa: usa la tua `.libs/` o `apt install libegl1`.

---

## Bonus — test UI mai eseguito  — RISOLTO

`bindings/python/tests/test_ui_smoke.py` saltava sempre il test di import UI:
la condizione `"PySide6" not in sys.modules` controlla i moduli *già
caricati*, non la disponibilità. Sostituita con `importlib.util.find_spec`:
ora il test gira davvero quando PySide6 c'è.

---

## Come applicare le correzioni alla repo

Opzione A — patch git (fornita accanto a questo zip):

```bash
cd CAD_zero
git apply CAD_zero_bug345_fix.patch
```

Opzione B — sostituire i file con quelli di questo zip.

## Verifica eseguita (ambiente di build: GCC, Ninja, Python 3.12)

| Verifica | Esito |
|---|---|
| Build C++ completo | 80/80 target |
| Test C++ (doctest) | **292/292** passati (1438 asserzioni) |
| Test funzionali Bug 1 (regressione) | extent 1.000 → 2.000 → 0.500 ✓ |
| Test funzionali Bug 2 (regressione) | STL 466 KB + OBJ 713 KB esportati ✓ |
| Test lifetime ndarray (Bug 4) | refcount +1, dati validi dopo gc ✓ |
| Identità UI installata (Bug 3) | byte-identica ad `app/src` ✓ |
| PySide6 `[ui]` extra (Bug 5) | PyOpenGL dichiarato ✓ |
| Sonda memoria (300 iterazioni) | RSS piatta, +0.00 MB ✓ |
| Smoke UI headless (MainWindow) | finestra + scena + export ✓ |
| pytest `bindings/python/tests` | **5/5** (prima 4 + 1 skip) |

## Comando di build

```bash
cmake -B build -G Ninja -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_POLICY_VERSION_MINIMUM=3.5 -DCAD_0_BUILD_PYTHON=ON \
  -DCAD_0_BUILD_TESTING=ON -DPython3_EXECUTABLE=$(which python3)
cmake --build build
./build/bin/CAD_0_tests        # suite C++
PYTHONPATH=build/python python -m pytest bindings/python/tests -q
PYTHONPATH=build/python python -m CAD_0.ui   # app desktop
```

---

## Aggiornamento post-push (regressione Bug 3)

**Cosa è successo**: applicando lo zip di correzioni sopra un checkout esistente
(estrarre → commit → push), i file *modificati* sono stati aggiornati, ma le 6
**eliminazioni** della UI stale (`bindings/python/CAD_0/ui/`) non si sono
propagate: i file assenti nello zip sono rimasti sul disco e sono stati
ricommitati. La UI stale era così tornata nel repository, ancora divergente
da `app/src/CAD_0/ui` (main_window 260 righe, viewport_widget 391, ecc.).

**Correzione**: `git rm -r bindings/python/CAD_0/ui/` + ri-commit.

**Retegrata di regressione** — la suite funzionale è ora **parte del repo**:

```bash
PYTHONPATH=build/<preset>/python python3 bindings/python/tests/test_bug_fixes.py
```

19 controlli su Bug 1-5 (editing parametri, export, lifetime ndarray,
assenza UI stale + identità byte con `app/src`, dipendenze dichiarate).
La CI la esegue a ogni build (step "Python bindings tests", richiede solo
numpy; i controlli UI degradano a skip senza PySide6/PyOpenGL).

**Nota CI**: il filtro branch `branches: [main]` in `ci-linux.yml` è sempre
stato corretto (un artifact del terminale diagnostico aveva mostrato
`ain]` — falsa pista, verificato con `od -c` sui byte grezzi).
