# Audit dei warning di compilazione — FASE A

Data: 2026-10-13
Base: `main` @ `0517286` ("correzione 3")
Scopo: censimento completo dei warning di compilazione su tutti i target,
prima di qualsiasi intervento (FASE B: fix; FASE C: `-Werror` sul kernel).
Nessuna modifica al sorgente è stata fatta in questa fase.

## Metodologia

* **gcc-release**: configure da directory vuota (cache CPM condivisa) +
  build completa — il log copre tutti gli 84 step di compilazione.
* **clang**: build pulita `--clean-first` con ricompilazione totale di
  tutti i target (log integrale; directory configure preesistente della
  sessione CI, stesso `CMakeLists.txt`, stesse versioni CPM). Una seconda
  configure da directory vuota (metodologia identica a gcc) ha riconfermato
  in modo indipendente tutti i warning first-party e terze parti dei target
  kernel prima di essere interrotta dai limiti di memoria della macchina
  (2 core / 3,9 GB, il TU `module.cpp` dei binding supera i 2 GB RAM da
  solo). I conteggi gcc sono risultati identici con entrambe le metodologie
  (249 righe, stessi siti), quindi il set clang sopra è stabile.

```
cmake --build --preset linux-clang --clean-first -- -j2   2>&1 | tee build-clang.log
cmake --build --preset linux-gcc-release -- -j2           2>&1 | tee build-gcc-release.log
```

Istogramma come da specifica (`grep 'warning:' | sed | sort | uniq -c |
sort -rn`), integrato con l'attribuzione di ogni warning al target in
compilazione (righe `[N/84] Building CXX object`).

Toolchain: clang 14.0.6 (stessa major della CI ubuntu-22.04), gcc 14.2.0
(Debian), cmake 4.4.4, ninja, Python 3.12.15, nanobind v2.2.0, xsimd
12.1.1, doctest v2.4.12 — sorgenti CPM identiche tra i due preset.

Nota: `-j2` invece del parallelismo ninja di default (nproc+2 = 4 job su
2 core / 3.9 GB RAM): con 4 job il build va in OOM e viene killato.

## Risultato clang (preset `linux-clang`)

**69 righe di warning, 24 siti unici** (52 righe terze parti / 17 first-party).

```
32  anonymous types declared in an anonymous union are an extension   [-Wnested-anon-types]
16  zero size arrays are an extension                                 [-Wzero-length-array]
 8  private field '...' is not used (8 campi distinti)                [-Wunused-private-field]
 6  implicit conversion from 'int' to 'float' may lose precision      [-Wimplicit-int-float-conversion]
 2  implicit conversion loses integer precision: 'uint64_t' to 'int'  [-Wshorten-64-to-32]
 2  implicit conversion turns floating-point number into integer      [-Wfloat-conversion]
 2  unused variable 'kEdgeTable' / 'kTriangleTable'                   [-Wunused-const-variable]
 1  unused function 'mesh_is_mostly_watertight'                       [-Wunused-function]
```

## Risultato gcc-release (preset `linux-gcc-release`)

**249 righe di warning, 30 siti unici** (242 righe terze parti / 7 first-party).

```
208  declaration of 'value'/'h'/'m'/'callback'/'set_self_py' shadows a member of
     'nanobind::...' (13 siti × 16 ripetizioni, header nanobind)      [-Wshadow]
 16  ISO C++ forbids zero-size array (nb_attr.h)                      [-Wpedantic]
 10  zero-size array member '...args' not at end of 'struct ...'      [-Wpedantic]
  6  conversion from 'int' to 'float' may change value (test)         [-Wconversion]
  4  conversioni nei .cpp di nanobind (uint32→bitfield, int→uint8)    [-Wconversion]
  3  shadow di locali nei .cpp di nanobind ('t', 'o', 'mt')           [-Wshadow]
  1  unused function 'mesh_is_mostly_watertight'                      [-Wunused-function]
```

## Confronto incrociato gcc ↔ clang (il dato più prezioso)

| Warning | clang 14 | gcc 14.2 | Lettura |
|---|---|---|---|
| int→float in `tests/unit/io/test_native.cpp:102-104` | 6 ([-Wimplicit-int-float-conversion]) | 6 ([-Wconversion]) | **Concordi** — warning first-party reali, indipendenti dal compilatore. Da chiudere con cast esplicito. |
| `mesh_is_mostly_watertight` inutilizzata | 1 | 1 | **Concordi** — helper morto dopo l'adozione di MC33. |
| 8 unused-private-field in `opengl_renderer.hpp` | 8 | 0 | **Solo clang**: diagnostica per-TU inesistente in gcc; i campi sono usati nel .cpp ma il warning scatta nelle altre TU che includono l'header. |
| Tabelle MC classiche in `mesh_extract.cpp:47/86` | 2 | 0 | **Solo clang**: gcc non attiva `-Wunused-const-variable` in C++ con `-Wall -Wextra`. Codice morto reale (sostituito da `mc33_tables.hpp`). |
| Shadow/pedantic negli header nanobind | 0 shadow, 16+32 estensioni | 235 righe | **Solo gcc**: GCC istanzia i template setter/getter di nanobind con `-Wshadow` attivo (parametro `value` omonimo del membro); clang non emette quelle diagnostiche ma segnala due estensioni (`-Wnested-anon-types`, `-Wzero-length-array`) che gcc tratta diversamente. Rumore di terze parti in entrambi i casi. |
| Conversioni dentro xsimd | 4 | 0 | **Solo clang**: `-Wshorten-64-to-32`/`-Wfloat-conversion` nel codice interno xsimd (to-int delle maschere, riduzioni). gcc non le segnala nei template. Terze parti. |
| Conversioni nei .cpp di nanobind | 0 | 4 (+3 shadow) | **Solo gcc**, nel target `nanobind-static` (terze parti, fuori dallo scope `-Werror` della FASE C). |

Prima delle build "vergini" era stata eseguita una build clean con le
directory configure in precedenza (sessione CI FASE 3): il conteggio era
identico (gcc 249/249 righe stesse; clang 69/69, stessi siti first-party)
— le cifre sopra sono stabili e riproducibili.

## Classificazione

### (a) Possibile bug reale: **ZERO warning**

Analisi dei tre vettori di rischio previsti dalla specifica:

1. **Shadowing first-party**: nessun warning `-Wshadow` in codice di primo
   parti su nessuno dei due compilatori (tutti gli shadow segnalati stanno
   negli header/sorgenti di nanobind).
2. **Promozioni/conversioni FP nei path di valutazione** (ADR-0014,
   `core/sdf/evaluate`, marching cubes): le uniche conversioni FP segnalate
   in first-party sono le 6 int→float in `tests/unit/io/test_native.cpp`,
   righe 102–104, dentro `sample_grid()` — il campionatore deterministico
   del test round-trip bit-identico. Non sono nel kernel: sono codice di
   test. Inoltre la conversione è matematicamente esatta: `i`, `j`, `k`
   valgono al più 8 (le chiamate usano `n = 7` e `n = 9`, righe 170–174) e
   ogni intero ≤ 2^24 è rappresentabile esattamente in `float` IEEE-754. Il
   cast esplicito previsto produce lo stesso valore bit-per-bit, quindi il
   test di equivalenza bit-identica non può cambiare esito. Il kernel
   (`evaluate.cpp`, `mesh_extract.cpp`, header `sdf/`) è pulito su
   entrambi i compilatori; le conversioni FP dentro xsimd stanno nel codice
   della libreria, non nel nostro path.
3. **Segni/size nei confronti di id**: nessun warning (`-Wno-sign-conversion`
   è una scelta preesistente del progetto; nessun `-Wsign-compare` scatta).

Conclusione: nessun bug latente è emerso dall'audit. Il kernel è già pulito
sotto il set di flag attuale (`-Wall -Wextra -Wpedantic -Wshadow
-Wnon-virtual-dtor -Wconversion -Wno-sign-conversion -Wno-unused-parameter`).

### (b) Benigni ma da chiudere esplicitamente: 17 clang / 7 gcc

| # | File:riga | Warning | Compilatori | Fix previsto (FASE B) | Perché il valore non cambia |
|---|---|---|---|---|---|
| 1-2 | `core/sdf/src/mesh_extract.cpp:47, 86` | `kEdgeTable` / `kTriangleTable` inutilizzate | solo clang | rimozione delle tabelle classiche (righe 43–599) | Simboli `static` mai referenziati: codice morto, rimpiazzato da `mc33_tables.hpp` (MC33). Nessuna unità di traduzione può osservarli. |
| 3-10 | `core/viewport/include/CAD_0/viewport/opengl_renderer.hpp:43–58` | 8 campi privati "not used" | solo clang | `[[maybe_unused]]` sulle dichiarazioni | I campi sono usati in `opengl_renderer.cpp`; il warning è per-TU (altre TU includono l'header). L'attributo è semanticamente neutro. |
| 11-16 | `tests/unit/io/test_native.cpp:102:52, 102:59, 103:52, 103:59, 104:52, 104:59` | int→float implicita | entrambi | `static_cast<float>(i)`, `static_cast<float>(n - 1)` | La promozione esplicita produce lo stesso `float` della conversione implicita (interi ≤ 8); ordine delle operazioni invariato. Garanzia: test round-trip bit-identici + golden file byte-identica. |
| 17 | `tests/unit/sdf/test_mesh_extract.cpp:70` | `mesh_is_mostly_watertight` definita e non usata | entrambi | rimozione della funzione (righe 62–94) | Funzione `static` senza call-site: codice morto dal passaggio a MC33. |

### (c) Rumore da terze parti: 52 clang / 242 gcc

| Sorgente | Consumatore first-party | Compilatori | Fix previsto (FASE B) |
|---|---|---|---|
| header nanobind (`nb_attr.h`, `nb_class.h`, `nb_types.h`, `nb_misc.h`, `stl/function.h`) — shadow/pedantic/estensioni | `_CAD_0` (6 TU) — e le stesse righe rivisitate in `nanobind-static` (terze parti) | gcc (235 righe) + clang (48) | `INTERFACE_SYSTEM_INCLUDE_DIRECTORIES` su `nanobind-static` dopo `nanobind_add_module()`: gli header diventano `-isystem` per i TU di `_CAD_0`. Nessuna modifica ai sorgenti nanobind. |
| header xsimd (`xsimd_sse2.hpp`, `xsimd_generic_math.hpp`) | `CAD_0_sdf` (`evaluate.cpp`) | solo clang (4) | `INTERFACE_SYSTEM_INCLUDE_DIRECTORIES` su `xsimd` in `core/sdf/CMakeLists.txt`. |
| `.cpp` di nanobind (target `nanobind-static`) | — (target di terze parti) | solo gcc (7) | Nessuno: target di terze parti, fuori dallo scope `-Werror` (FASE C lo applica solo a kernel + test). I suoi flag derivano dalle `add_compile_options` globali e non vengono toccati. |
| `third_party/robust_predicates` (header `predicates.c.h`) | `CAD_0_math` (PUBLIC) e `CAD_0_tests` | 0 warning su entrambi | Profilassi FASE C: include come SYSTEM (`-isystem`) sui target che le consumano, così `-Werror` sui test non può rompersi su diagnostica futura di codice Shewchuk. |
| doctest (`doctest/doctest.h`) | `CAD_0_tests` | 0 warning su entrambi | Profilassi FASE C, stesso trattamento SYSTEM. |

## Caveat

1. **gcc locale 14.2 vs gcc CI (ubuntu-22.04 → gcc 11/12)**: il gcc locale è
   più recente e più loquace di gcc 11; la bonifica è a prova di
   compilatore-versione (cast espliciti, `[[maybe_unused]]`, `-isystem`),
   ma la run CI reale (gcc 11) resta la verifica finale. I falsi positivi
   documentati GCC 12+ (`stringop-overflow`) riguardavano header oneTBB
   (disattivato: `CAD_0_USE_TBB=OFF`) e non compaiono in nessuno dei due set.
2. **nanobind-static** eredita i flag warning globali (`add_compile_options`
   al root) e su gcc produce 7 warning nel proprio codice: è un target di
   terze parti, non un target di primo parti — la FASE C non lo tocca e il
   goal "zero warning sui target di primo parti" resta soddisfatto.
3. I flag `-Wno-unused-parameter` e `-Wno-sign-conversion` sono scelte
   preesistenti documentate in `CMakeLists.txt` e restano invariate:
   l'audit fotografa il set di flag attuale.
4. Il preset `linux-gcc-debug` (CI) non è nell'audit per specifica
   (clang + gcc-release); verrà verificato integralmente in FASE C con
   l'intera suite su tutti e tre i preset.

## Piano operativo FASE B (a comportamento invariato)

1. `tests/unit/io/test_native.cpp` — 6 cast espliciti (tabella (b), righe
   11–16). Equivalenza: stessa promozione, stessi float, stesso ordine.
2. `core/sdf/src/mesh_extract.cpp` — rimozione tabelle morte.
3. `core/viewport/include/CAD_0/viewport/opengl_renderer.hpp` —
   `[[maybe_unused]]` su 8 campi.
4. `tests/unit/sdf/test_mesh_extract.cpp` — rimozione helper morto.
5. `bindings/python/CMakeLists.txt` — nanobind come SYSTEM per `_CAD_0`.
6. `core/sdf/CMakeLists.txt` — xsimd come SYSTEM per `CAD_0_sdf`.
7. `core/math/CMakeLists.txt`, `tests/CMakeLists.txt` — robust_predicates
   e doctest come SYSTEM (profilassi `-Werror`).

Garanzia di equivalenza (specifica FASE B): dopo i fix devono restare verdi
e invariati i test round-trip bit-identici e il golden file
`tests/golden/native_v0_1_0.CAD_0` (re-serializzazione byte-identica);
suite doctest completa + test Python dei bindings verde su tutti i preset.
Nessun `-Wno-*` globale nuovo; eventuali soppressioni puntuali solo con
`#pragma diagnostic push/pop` e commento (non previste al momento).
