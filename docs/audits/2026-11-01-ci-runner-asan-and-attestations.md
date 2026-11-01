# La CI che non può mentire — il runner attuale, l'ASan dei binding, le attestazioni allineate

Data: 2026-11-01
Base: `main` @ `f7b4c06` (H2-u6 morso 6)
Scope: **Consegna B m1** — infrastruttura CI e registro. Il morso non
tocca un byte di logica: la prova è il rito locale integro (v. ultima
sezione) e i golden intatti.

Il quadro: GitHub ha avviato il pensionamento dei runner 22.04. La CI è
l'arbitro di tutto ciò che il progetto ha costruito — H1, H2, il
naming: se lei si ferma, si ferma il metodo.

## B1 — Il runner e la toolchain: l'asse che non si nega

**La mossa.** `ci-linux.yml` passa a `ubuntu-24.04`; le colonne sono
**gcc 14 e clang 19**, pinne nei preset stessi (CMakePresets.json):
`linux-gcc-debug` e `linux-gcc-release` portano `g++-14`, `linux-clang`
porta `clang++-19` (anche `linux-python` si allinea a `g++-14`). Il
preset È la toolchain: una sola fonte di verità, la stessa per il rito
locale e per la CI — la CI non può essere più vecchia del rito, mai il
contrario.

**Il runner 24.04, verificato.** L'immagine porta g++-14 14.2.0 ma NON
clang 19 (porta 16/17/18; l'aggiunta di clang 19 come default, annunciata
in actions/runner-images#11895, è stata chiusa *not planned* il
2025-04-07). Il workflow quindi installa `clang-19` esplicitamente
(noble-updates: 19.1.1) insieme a `g++-14`; lo step "Compiler versions"
stampa i compilatori delle colonne in ogni job, come il contratto
chiede.

**Il rito locale coincide per serie.** gcc 14.2.0 (Debian trixie) e la
colonna CI gcc 14.2.0 (Ubuntu noble): stessa serie. clang 19.1.7
(Debian) e la colonna CI clang 19.1.1 (Ubuntu): stessa serie. Il
doppio compilatore locale e la CI parlano la stessa lingua.

**Il valore della colonna attuale, misurato.** I rossi clang-only della
fase H2-u3 (eps_p_of, kTwoPi_f — la storia nelle commit `98210a5` e
`6525fa0`) avevano dimostrato che le differenze tra release del
compilatore sono il radar gratuito. Le release nuove non hanno trovato
nulla da dire: gcc 14 (release e debug+ASan) e clang 19 compilano il
kernel e la suite con **zero warning first-party**. Le righe che restano
(157 su gcc, 30 su clang) sono TUTTE dentro nanobind-static — terze
parti, fuori dallo scope `-Werror`, la nota storica dell'audit warning.
`-Werror` resta dov'era: nessun flag toccato.

**Il preset `linux-gcc11-ci` è ritirato.** La sua ragione d'essere — la
parità con la colonna gcc 11 di ubuntu-22.04 — non esiste più. Il
contratto di toolchain (docs/audits/2026-10-13-warning-audit.md) è
aggiornato lì dove sta (sezione in coda a questo documento).

**Il gate Python 3.10, trasportato, non perso.** Il `python3` di sistema
del 22.04 ERA il gate: ogni modulo Python del repo veniva importato da
un interprete 3.10 vero, nei test. Il 24.04 porta 3.12: un gate che
sparisce in silenzio è una menzogna per omissione. Il floor ha quindi un
suo job (`python-3-10-floor`: setup-python 3.10 + `compileall` su
app/bindings/examples/scripts/tests — il livello sintassi, la stessa
classe che il vecchio runner catturava all'import). Il gate a RUNTIME
(i test eseguiti su 3.10) si ritira col runner: **registrato** qui, col
prezzo di ripristino — una colonna della matrice che compila i binding
contro un interprete 3.10 (setup-python lo permette: ha gli header) —
aperta quando un caso reale lo chiede.

## B2 — L'ASan dei binding: il gap dichiarato, chiuso

**Il debito.** Il report della correzione H1 lo dichiarò: i test dei
binding Python girano SENZA AddressSanitizer — la superficie più
esposta (nanobind, i ponti di memoria) era la meno protetta.

**La chiusura.** Nuovo target ctest `CAD_0_python_bindings`
(tests/CMakeLists.txt + scripts/run_python_bindings_suite.py): la suite
dei binding — lo standalone Bug 1-5 più i 192 test pytest non-GL — corre
dentro ctest, in **ogni job della matrice**. La prova che il gap è
chiuso è la riga della matrice (`2/2 Test #2: CAD_0_python_bindings`),
non una frase. Nel job strumentato (linux-gcc-debug: ASan/UBSan) il
launcher preleva la libasan dalla build stessa e la PRECARICA: lo stesso
pattern già usato per il C++ — la stessa build strumentata, il nuovo
target che la consuma — ora dal lato Python.

**La scoperta — il sanitizer ha trovato qualcosa, della classe per cui
il gap esisteva.** Al primo lancio del target, la prima eccezione C++ che
attraversa il ponte muore con:

```
AddressSanitizer: CHECK failed: asan_interceptors.cpp:458
    "((__interception::real___cxa_throw)) != (0)"
    #4 throw_empty_surface core/bridge/src/sdf_to_brep.cpp:32
```

Python è un host C: non carica libstdc++. Quando il modulo viene
dlopenato e la prima `std::invalid_argument` attraversa il ponte,
l'intercettore `__cxa_throw` di ASan non ha il simbolo reale da chiamare
— la correzione H1 non lo aveva mai visto perché i suoi binding giravano
senza sanitizer: era ESATTAMENTE il difetto di copertura dichiarato.
**Corretta nel launcher**: anche il runtime C++ viene precaricato,
risolto dal DT_NEEDED del modulo stesso via `ldd` (verità di fatto, non
configurazione). Il contratto di messaggio dell'eccezione è sopravvissuto
intatto al sanitizer — `test_empty_input_original_exception_message` è
la regressione: il testo ORIGINALE della diagnosi, prefisso e rimedio
inclusi.

**Il verdetto del sanitizer sui ponti.** Zero findings sull'intera
suite sotto ASan+UBSan: nessun report AddressSanitizer, nessun
`runtime error` di UBSan, 192 test + Bug 1-5 verdi in 51 s.
`detect_leaks=0` è dichiarato e motivato nel launcher: a shutdown
l'interprete trattiene le sue arene e PySide6 i suoi singleton — quel
rumore non è nostro; use-after-free, overflow e le vite dei ponti (la
classe dichiarata) restano armati.

**I due file GL** (test_ui_navigation, test_ui_startup) restano al loro
step Xvfb nel workflow: un display è il loro prerequisito, non un
sanitizer — la stessa divisione che la CI ha sempre avuto, non una
perdita di copertura.

## B3 — Le attestazioni Windows: il muro onesto

**Lo stato verificato.** CONTRIBUTING dichiarava "a PR is not
mergeable until ctest passes on Linux and Windows CI" — ma NON ESISTE
nessuna colonna Windows: la CI giudica su Linux soltanto. Un'attestazione
non testata non è supporto — è il difetto di onestà di piattaforma che
questa consegna esiste per chiudere.

**La derivazione (a), la prova della colonna: non eseguibile onestamente
da questo rito.** Non c'è MSVC nell'ambiente locale; spingere una
colonna mai compilata violerebbe la regola della consegna stessa (CI
tutta verde) trasformando l'attestazione in una scommessa col runner.

**La derivazione (b), quindi — onesta e in un tempo solo.** Le
attestazioni si correggono ORA alla verità:

- CONTRIBUTING e README: la frase diventa "Linux — CI-tested (GCC and
  Clang columns); Windows: registered, priced, its own unit when a real
  case arrives" (con il puntatore a questo documento);
- pyproject.toml: il classifier `Operating System :: Microsoft ::
  Windows` è rimosso — un attestato di supporto che la CI non testimonia;
  resta `POSIX :: Linux`, che la CI testimonia;
- istruzioni.md: le sezioni Windows (build MSVC, binding, il recipe
  ci-windows.yml d'esempio) restano come MATERIALE dell'unità registrata,
  ma il loro stato è dichiarato in testa alla sezione.

**L'unità registrata, col costo visto** (dall'inventario del repo — non
da un collaudo, che non c'è stato):

1. la conformità C++20 di MSVC sotto `/permissive-` non è mai stata
   provata: il codice è nato e cresciuto su gcc/clang;
2. il pin `/Wv:19.40` in CMakeLists.txt presuppone MSVC ≥ 19.40
   (VS 17.10) — da verificare contro il toolset di windows-latest;
3. nanobind + MSVC + i dev header di Python sul runner;
4. lo step UI/Xvfb è Linux-shaped: su Windows servirebbe un harness GL
   equivalente (la suite UI è headless-first, ADR-0017, ma sotto
   offscreen EGL il collaudo va rifatto);
5. i golden byte-identical (ADR-0014) non sono mai stati osservati su
   MSVC (`/fp:strict` è già in posto, la prova manca);
6. il target ctest `CAD_0_python_bindings` si registra su Linux
   (tests/CMakeLists.txt): il suo launcher è POSIX — l'unità Windows
   dovrà decidere la sua forma lì.

Il preset `windows-msvc` resta in CMakePresets.json: è il punto
d'ingresso dell'unità. **Criterio d'ingresso: un caso reale** — un
utente, un contributore, una dipendenza che ne chieda il supporto. Il
metodo del muro onesto applicato alle piattaforme.

## B4 — I due assi di versione: due semantiche, due changelog

**Lo stato.** Il software si dice 0.1.0 (CMakeLists `project(VERSION)`,
pyproject `version`, `CAD_0.__version__`); il FORMATO è v0.4.0
(`kNativeVersion` in native.hpp, la serie ADR-0007) — e nessuna fonte
diceva al lettore quale dei due assi sta parlando. Il tag
`v0.1.0-alpha` (software) e il primo anello della catena del formato
(v0.1.0) condividono lo stesso numero per caso storico: la collisione è
il luogo della confusione.

**La separazione, documentata.** Nuovo paragrafo in CONTRIBUTING ("The
two version axes"): chi parla cosa — i tag git e le release notes
parlano del software; i bump `v0.x` e la catena dei migratori parlano
del formato; i due assi si muovono indipendenti. I punti ambigui del
README sono etichettati col loro asse (il bullet del formato negli
Highlights; il tag `v0.1.0-alpha` nella sezione NOTICE).

**Nessun bump di nessuno dei due** — si allineano le parole, non i
numeri: il v0.5.0 del formato è registrato per il fillet (ADR-0036 D6
lo ha già deciso).

**Altro allineato lungo la stessa via.** istruzioni.md contava "165
test" nel runner headless: il conto vero oggi è 772 casi doctest + 205
test pytest (+ la standalone Bug 1-5). La frase ora non fissa un numero
che invecchia a ogni unità.

## Il rito locale (la prova che nulla si è rotto)

Eseguito per intero sui compilatori delle nuove colonne, prima del
commit:

| passo | verdetto |
|---|---|
| `linux-gcc-release` (g++-14, RelWithDebInfo) | build pulita, 0 warning first-party; ctest **2/2** (`CAD_0_tests` 772 casi + `CAD_0_python_bindings`) |
| `linux-gcc-debug` (g++-14, Debug + ASan/UBSan) | build pulita; ctest **2/2** — i binding SOTTO il sanitizer (51 s), zero findings |
| `linux-clang` (clang++-19, RelWithDebInfo) | build pulita, 0 warning first-party (30 righe, tutte nanobind-static); ctest **2/2** |
| UI smoke GL sotto Xvfb (i due file) | **13 passed** |
| Python 3.10 floor (`compileall`, python 3.10.22) | tutto l'albero compila |
| `scripts/check_fstring_backslashes.py` | 55 file, nessun backslash in f-string |
| `scripts/test_check_commit_subject.py` | 67 casi, 0 falliti |
| golden | `git status tests/golden/` vuoto |

## Verificato ma non corretto

- **Il rumore nanobind a shutdown** ("nanobind: leaked function ..."
  all'uscita dell'interprete, poi "this is likely caused by a reference
  counting issue in the binding code"): informativo, exit code 0,
  preesistente — visibile in ogni run CI da sempre. Il sweep di
  consolidamento 2026-10-16 aveva già isolato la parte che era dei
  globali del file di test (v. il docstring di test_bug_fixes.py); le
  funzioni del modulo che restano vive a shutdown sono la classe
  rimanente. Fuori dallo scope di questo morso (non un byte di logica):
  registrato qui perché ora, col target ASan, è sotto gli occhi di ogni
  run.
- **Le righe di warning terze parti in nanobind-static** (157 gcc / 30
  clang): la nota storica dell'audit warning, invariate — il target è
  di terze parti, fuori dallo scope `-Werror`.
