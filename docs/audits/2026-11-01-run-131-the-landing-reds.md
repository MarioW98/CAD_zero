# I rossi dell'atterraggio — la run #131: il subject incollato, l'ICE del gcc di Ubuntu

Data: 2026-11-01
Base: `main` @ `6cc5d59` (Consegna B m1, l'atterraggio)
Scope: **la correzione mirata dell'atterraggio** — la run #131 è rossa
e il morso esiste per trovare ciò che il rosso sta dicendo. La regola
di ferro tenuta per tutto il morso: il rosso non si cura indebolendola
— nessun job tolto dalla matrice, nessun `-Wno-*`, nessun ritorno al
22.04, nessun warning zittito.

## Il verdetto della run (il log è il giudice)

Run #131 su `6cc5d59` (3m 42s), tre job rossi, due verdi:

| job | verdetto | durata |
| --- | --- | --- |
| subject-lint | **failed** | 5s |
| build-and-test (linux-gcc-debug) | **failed** | 2m 56s |
| build-and-test (linux-gcc-release) | **failed** | 2m 48s |
| python-3-10-floor | passed | — |
| build-and-test (linux-clang) | passed | 2m 40s |

Due classi distinte, entrambe riprodotte in locale prima della fix.

## Rosso 1 — il subject dell'atterraggio (subject-lint)

**L'errore, dal giudice locale (stessa funzione, stesso range della
CI):**

```
FAIL: 'ci: the runner current, the bindings under ASan, the claims and
version axes aligned (Consegna B m1) Rebased sull'origin/main attuale
(4a23c1f, il morso 6 squashed da te) — applicazione pulita verificata.'
— un hash git dentro il soggetto: git identifica, il soggetto deve
  descrivere
```

**La classe: l'applicazione, non la regola.** Il soggetto di
`6cc5d59` porta incollata una riga del README di consegna ("Rebased
sull'origin/main attuale (4a23c1f, ...)") — e con essa l'hash
`4a23c1f`, che la regola 5 boccia. La stessa operazione ha perso il
corpo B1–B5 del commit originale. L'hook `commit-msg`
(scripts/git-hooks) avrebbe bloccato il soggetto PRIMA del commit:
l'attraversamento che ha incollato la riga non è passato dall'hook.

**La correzione, del giusto lato.** Il soggetto incollato è storia
pushata: non si riscrive (push normale, mai force). Il commit
d'atterraggio di QUESTA correzione porta il soggetto pulito, e la CI
giudica il range `6cc5d59..HEAD` — solo i commit nuovi: il verdetto
dipende dal nuovo soggetto, non dalla storia. Il subject-lint torna
verde con la forma buona, senza toccare né la regola né il gate.

## Rosso 2 e 3 — l'ICE del gcc di Ubuntu (le due colonne gcc)

**L'errore, dalla job rossa (primo step rosso: Build), citato:**

```
tests/unit/brep/test_g1_unit3_extrude.cpp: In function
‘void DOCTEST_ANON_FUNC_24()’:
tests/unit/brep/test_g1_unit3_extrude.cpp:994:1: internal compiler
error: in gimplify_var_or_parm_decl, at gimplify.cc:3308
```

**La classe: (a) colonna build — ma la firma non è un warning: è un
bug del compilatore, acceso dalla forma di tre chiamate.** Non è
nemmeno la classe b (il target ASan dei binding non è mai stato
raggiunto: il rosso è a Build, prima di Test) né la classe c (lo step
"Install dependencies" è verde: i pacchetti ci sono tutti; la colonna
clang, con lo stesso workflow, è verde fino in fondo).

**La riproduzione locale, col metodo dei due rossi clang precedenti
(ripro = la verità).** Il gcc del rito locale è il 14.2.0-19 di Debian
trixie; quello della CI è il `g++-14` che apt risolve su
ubuntu-24.04: **14.2.0-4ubuntu2~24.04.1** (noble-updates/universe).
Stessa serie 14.2.0 — l'assunzione "stessa serie" dell'audit di m1 è
l'assunzione che questo rosso ha falsificato: il livello di patch
della distro conta. Il toolchain esatto di Ubuntu è stato scaricato in
userland (pool di archive.ubuntu.com, `apt-get download` +
`dpkg-deb -x`, senza root: gcc-14-base, gcc-14, g++-14, cpp-14,
libgcc-14-dev, libstdc++-14-dev, libasan8, libubsan1 — tutti
14.2.0-4ubuntu2~24.04.1) e la colonna è stata costruita con lui:

| toolchain | il TU `test_g1_unit3_extrude.cpp` | verdetto |
| --- | --- | --- |
| Ubuntu gcc 14.2.0-4ubuntu2~24.04.1 (CI) | **ICE** gimplify.cc:3308 | rosso |
| Debian gcc 14.2.0-19 (rito locale) | compila pulito | verde |
| clang 19.1.7 | compila pulito | verde |

**Il riproduttore minimale (24 righe, senza intestazioni del
progetto):** un aggregato con inizializzatori designati, annidato
nella doppia graffa di un argomento `const std::vector<T>&` —
`take({{RegionSpec{.window = true, .umin = a0, ...}}}, {{a1, 1.0f}})`
— accende l'ICE sull'Ubuntu gcc ed è accettato da Debian gcc e clang.
Il bisect del TEST_CASE (a blocchi) ha isolato il grilletto nella
chiamata della finestra d'arco; la scansione del codebase
(`\{\{\s*\w+\s*\{\s*\.`) ha trovato **tre siti** con la stessa forma,
tutti in questo file: la finestra d'arco del settore (il primo che il
compilatore incontra), la finestra periodica del cerchio e la parete
del foro dell'anello (sarebbero emersi uno alla volta, fissato il
precedente — il compilatore muore al primo).

**La fix, della classe giusta: il codice, mai il flag.** I tre
aggregati designati passano per **locali nominate** (`arc_wall`,
`rim_window`, `hole_window`) e la chiamata consuma `{nome}` — la forma
che il TEST_CASE stesso già usava per le regioni cap (`cap_lo_region`,
`cap_hi_region`) e che su tutte le colonne compila. Semantica
identica: stesse inizializzazioni, stessi valori, stesse asserzioni;
il file passa da 772 casi dichiarati a 772 casi dichiarati. Il
commento sul primo sito qualifica il perché (la firma dell'ICE, i due
toolchain, il rimando a questo documento); gli altri due siti
rimandano lì. Nessun `-Wno-*`, nessun flag toccato, `-Werror` resta,
il job resta nella matrice.

**Il registro residuo.** Il rito locale non può specchiare il livello
di patch della distro CI: Debian 14.2.0-19 e Ubuntu
14.2.0-4ubuntu2 sono la stessa serie con packaging diversi, e questo
rosso è esattamente quel delta. La lezione del doppio compilatore,
secondo capitolo: **quando una colonna gcc è rossa in CI e il rito
locale è verde, il primo passo è ricostruire col toolchain pinned di
Ubuntu in userland** (la ricetta: pool di archive.ubuntu.com +
`dpkg-deb -x`, nessun root richiesto) — non partire a caccia alla
cieca. Il riproduttore delle 24 righe resta qui come taratura del
radar.

## Il rito locale completo, prima del commit (la firma dell'atterraggio pulito)

Cinque colonne, tutte verdi in prima persona:

- **gcc 14.2.0-19 Debian, debug + ASan/UBSan**: ctest 2/2 (il target
  `CAD_0_python_bindings` sotto sanitizer: ~51 s, zero findings)
- **gcc 14.2.0-19 Debian, release**: ctest 2/2 (~13 s)
- **clang 19.1.7, RelWithDebInfo**: ctest 2/2 (~13 s)
- **gcc 14.2.0-4ubuntu2~24.04.1 Ubuntu (il toolchain della CI),
  release**: build pulita (zero warning first-party, -Werror attivo),
  ctest 2/2 (~14 s)
- **gcc 14.2.0-4ubuntu2~24.04.1 Ubuntu, debug + ASan/UBSan**: build
  pulita, ctest 2/2 (binding sotto sanitizer ~51 s, zero findings)

Più: UI GL sotto Xvfb 13 passed; doctest 772 casi; golden e naming
digest dentro la suite, intatti; `check_fstring_backslashes` 56 file
puliti; self-test del subject checker 67/67; range `6cc5d59..HEAD`
conforme al checker.

## Cosa NON è cambiato

La suite non perde un byte di semantica: le 772 asserzioni del file
toccato sono le stesse, cambia solo la forma di tre chiamate (il
contenuto degli aggregati è identico). Il workflow non cambia una
riga: le colonne, il target ASan dei binding, il floor 3.10 e il
subject-lint restano esattamente come l'atterraggio li ha portati —
era la CI ad avere ragione, entrambe le volte.
