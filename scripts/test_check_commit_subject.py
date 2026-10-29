#!/usr/bin/env python3
"""Test del checker dei soggetti, attraverso la sua stessa superficie.

Il checker (scripts/check_commit_subject.py) espone --explain: classifica
una stringa e stampa PASS/FAIL col motivo. Questo test usa ESATTAMENTE
quel binario — niente importazione di internals: ogni giudizio passa
dalla stessa via a riga di comando che usano l'autore a mano, l'hook
commit-msg (stessa funzione check_subject, via --commit-msg) e la CI
(range pushato). Pinna:

  1. i marker vuoti ITALIANI: la run #93 ("correzione") era caduta
     sulla LUNGHEZZA (10 < 15) solo per caso — ora il marker cade come
     marker, anche quando la lunghezza basterebbe ("correzione
     interfaccia 1" ha 24 caratteri ed e' un soggetto reale in storia);
  2. le regole pre-esistenti (starter inglesi, etichette di progresso,
     tag puri): l'estensione non le ha modificate;
  3. la forma buona: il conventional commit passa in ogni case
     ("fix(lint): ...", "FIX(lint): ..." — la parentesi non matcha lo
     starter) e la parola italiana DENTRO il soggetto non c'entra (la
     regola e' sullo START);
  4. il case e la forma Unicode non cambiano il difetto: i MAIUSCOLI
     ("UNITÀ 4", "FIX", "BOZZA", "CORREZIONE") cadono come i gemelli
     minuscoli, la "à" decomposta come la precomposta, i tag/percorsi/
     nomi di file maiuscoli come i minuscoli. Gli escape capitalizzati
     storici ("Fix f-string Python 3.10", "Correzione del ...") muoiono
     qui: la lista storica pass/fail si aggiorna — e' il comportamento
     giusto — perche' il difetto resta esattamente quello.

Uso:
  python3 scripts/test_check_commit_subject.py

Exit code: 0 se ogni caso esce come atteso, 1 altrimenti.
"""

import subprocess
import sys
import unicodedata
from pathlib import Path

CHECKER = Path(__file__).resolve().parent / "check_commit_subject.py"

# (soggetto, conforme atteso, motivo atteso se non conforme)
# Il motivo e' una sottostringa dell'output FAIL del checker; per i
# casi conformi e' None. I riferimenti alle run sono ai soggetti veri
# che le hanno rese rosse.
CASES = [
    # --- 1. marker vuoti italiani: cadono COME MARKER -----------------
    # "correzione" e' il soggetto esatto della run #93.
    ("correzione", False, "starter vuoto"),
    ("correzioni", False, "starter vuoto"),
    ("sistemazione", False, "starter vuoto"),
    ("sistemazioni", False, "starter vuoto"),
    ("modifica", False, "starter vuoto"),
    ("modifiche", False, "starter vuoto"),
    ("aggiornamento", False, "starter vuoto"),
    ("aggiornamenti", False, "starter vuoto"),
    # anche quando la lunghezza basterebbe — e' questo il punto della
    # fix: prima passavano, il marker era un buco:
    ("correzione 3", False, "starter vuoto"),                    # in storia
    ("correzione interfaccia 1", False, "starter vuoto"),        # in storia, 24 char
    ("sistemazione dell'hook commit-msg locale", False, "starter vuoto"),
    ("aggiornamento delle dipendenze di build", False, "starter vuoto"),
    ("correzione/fix vari", False, "starter vuoto"),             # forma 'a/b'
    # --- 2. regole pre-esistenti: regressione -------------------------
    ("fix", False, "starter vuoto"),
    ("update", False, "starter vuoto"),
    ("various fixes", False, "starter vuoto"),
    ("bozza sketch", False, "etichetta di progresso"),           # run #84
    ("intro kernel 2d", False, "etichetta di progresso"),        # run #90, 15 char
    ("Unità 2 sketch", False, "etichetta di progresso"),         # run #91
    ("unità 2 fix e unità 3", False, "etichetta di progresso"),  # run #92
    ("Riepilogo:", False, "prefisso tipo mancante o malformato"),   # run #85, 10 char
    # la regola lunghezza si giudica da sola su un soggetto con la
    # forma giusta (il prefisso): 13 caratteri < 15:
    ("feat(brep): x", False, "troppo corto"),
    ("U2", False, "tag di unita'"),                              # run #89
    ("2.3", False, "versione numerica nuda"),
    ("docs/adr", False, "percorso di file"),
    ("0027-phase-f-boolean-feature-slice.md", False, "nome di file"),
    # --- 3. la forma buona passa ---------------------------------------
    ("fix(lint): ban empty-marker subjects in Italian too", True, None),
    ("docs: revisione ADR-0030 D5 dopo review", True, None),
    # la parola italiana dentro il soggetto non c'entra: la regola e'
    # sullo START, non sulla presenza:
    ("docs: aggiornamento del contratto ADR-0029", True, None),
    # --- 4. MAIUSCOLI e forma Unicode: il difetto non ha case ----------
    # la riproduzione della fix: "UNITÀ 4 DI ADR-0030 — test" con la
    # "À" DECOMPOSTA (A + accento combinato, quella che certi editor
    # incollano) passava tutte le regole; la forma precomposta era
    # gia' presa dal (?i). Ora NFC+casefold le giudica uguali:
    ("UNITÀ 4", False, "etichetta di progresso"),
    ("UNITÀ 4 DI ADR-0030 — test", False, "etichetta di progresso"),
    (unicodedata.normalize("NFD", "UNITÀ 4 DI ADR-0030 — test"),
     False, "etichetta di progresso"),
    ("BOZZA", False, "etichetta di progresso"),
    ("BOZZA del pannello proprietà del foro", False, "etichetta di progresso"),
    ("INTRO del canvas dei gesti D5", False, "etichetta di progresso"),
    ("FIX", False, "starter vuoto"),
    ("FIX della regressione del lint dei soggetti", False, "starter vuoto"),
    ("CORREZIONE", False, "starter vuoto"),
    ("CORREZIONE dell'interfaccia di estrusione", False, "starter vuoto"),
    ("Update dei golden del kernel", False, "starter vuoto"),
    ("Sistemazione dei test della UI", False, "starter vuoto"),
    # la natura non ha case nemmeno per versioni, percorsi e nomi di
    # file: "0027-PHASE-F-BOOLEAN.MD" (23 caratteri) prima passava
    # perche' l'estensione era minuscola:
    ("V1.2", False, "versione numerica nuda"),
    ("DOCS/ADR", False, "percorso di file"),
    ("0027-PHASE-F-BOOLEAN.MD", False, "nome di file"),
    # e la forma buona non ha case: il conventional maiuscolo passa
    # (i due punti non matchano lo starter, in ogni case):
    ("FIX(lint): subject rules are case-insensitive", True, None),
    # gli escape capitalizzati STORICI muoiono qui: "Fix f-string
    # Python 3.10" e' un soggetto reale della storia, ora non conforme
    # come il suo gemello minuscolo — la lista storica si aggiorna:
    ("Fix f-string Python 3.10", False, "starter vuoto"),
    ("Correzione del calcolo AABB del cilindro", False, "starter vuoto"),
    # --- 5. H2 Parte C: il prefisso tipo conventional ------------------
    # "scripts:" non e' nel vocabolario dei tipi: la forma buona ha
    # il tipo conventional (feat/fix/docs/chore/test/...) — il vecchio
    # esempio PASS della lista 3 si aggiorna, e' il comportamento
    # giusto (la lista storica pass/fail si aggiorna anche qui):
    ("scripts: hook commit-msg tracciato eseguibile",
     False, "prefisso tipo mancante o malformato"),
    # i due soggetti reali della scivolata H1, bocciati dalla macchina:
    ("g2 - dimenticanza file", False, "prefisso tipo mancante o malformato"),
    ("Commit 1/4 — c851bdc · docs (PARTE A) docs: ADR-0034 accepted",
     False, "numerazione di consegna"),
    # il prefisso esiste ma il tipo che tocca codice senza scope:
    ("feat: the boolean input gate lands with named errors",
     False, "scope mancante per il tipo 'feat'"),
    ("fix: the weld straddle regression",
     False, "scope mancante per il tipo 'fix'"),
    # i tipi di registro restano leciti senza scope (la forma storica):
    ("chore: restore the commit-msg hook executable bit", True, None),
    ("test(lint): pin the subject checker judgments via --explain", True, None),
    # il case non cambia la forma buona ne' la cattiva:
    ("FEAT(brep): the boolean input gate lands",
     True, None),
    ("Feat(BREP): the boolean input gate lands",
     True, None),
    # --- 6. H2 Parte C: i divieti di contenuto --------------------------
    # la numerazione di consegna, anche dentro un involucro conforme:
    ("feat(brep): Commit 1/4 — the input gate",
     False, "numerazione di consegna"),
    ("docs: commit 3 of the ADR sweep",
     False, "numerazione di consegna"),
    # l'hash git nel soggetto (0x escluso — il dato esadecimale
    # dichiarato resta lecito):
    ("feat(brep): re-pin the goldens after c851bdc",
     False, "un hash git dentro il soggetto"),
    ("feat(brep): the seed 0xa11ced01 oracle pins the weld",
     True, None),
    # il banner di titolo di prompt:
    ("feat(brep): the input gate lands =====================",
     False, "titolo di prompt"),
    ("docs: ADR-0034 accepted ══════════════════",
     False, "titolo di prompt"),
    # il maiuscole-urlò (misura sul soggetto originale):
    ("feat(brep): THE BOOLEAN INPUT GATE LANDS WITH THE NAMED ERRORS",
     False, "maiuscole-urlò"),
    ("feat(brep): the input gate — ADR-0034 D3 and the SSI table pinned",
     True, None),
    # i due soggetti conformi di QUESTA consegna, giudicati dalla
    # macchina che li fermerebbe prima del commit:
    ("feat(brep): the boolean input gate — one validator enforces the declared class with named errors",
     True, None),
    ("feat(brep): the boolean output gate — Layer-1 postconditions hard, the open-shell union refused",
     True, None),
    ("feat(ci): the subject lint extended — typed prefixes, delivery counters, SHAs and prompt banners refused",
     True, None),
]


def explain(subject: str) -> tuple[int, str]:
    """Manda il soggetto al checker via --explain; ritorna (exit, output)."""
    proc = subprocess.run(
        [sys.executable, str(CHECKER), "--explain", subject],
        capture_output=True, text=True, encoding="utf-8", check=False,
    )
    return proc.returncode, proc.stdout.strip()


def main() -> int:
    failures = 0
    for subject, conforming, motivo in CASES:
        code, out = explain(subject)
        if conforming:
            ok = code == 0 and out.startswith("PASS:")
        else:
            ok = code == 1 and out.startswith("FAIL:") and motivo in out
        mark = "ok " if ok else "XX "
        expected = "PASS" if conforming else "FAIL"
        print(f"[{mark}] {expected} atteso  {subject!r}")
        if not ok:
            failures += 1
            print(f"        ottenuto: exit {code}, {out!r}")
            if not conforming and motivo is not None and motivo not in out:
                print(f"        motivo atteso assente: {motivo!r}")
    print(f"\ncasi: {len(CASES)}  falliti: {failures}")
    if failures:
        print("TEST ROSSO: il checker non giudica come questo test pinna.")
        return 1
    print("TEST VERDE: ogni caso esce come pinna (via --explain).")
    return 0


if __name__ == "__main__":
    sys.exit(main())
