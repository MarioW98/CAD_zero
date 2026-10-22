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
    ("Riepilogo:", False, "troppo corto"),                       # run #85, 10 char
    ("U2", False, "tag di unita'"),                              # run #89
    ("2.3", False, "versione numerica nuda"),
    ("docs/adr", False, "percorso di file"),
    ("0027-phase-f-boolean-feature-slice.md", False, "nome di file"),
    # --- 3. la forma buona passa ---------------------------------------
    ("fix(lint): ban empty-marker subjects in Italian too", True, None),
    ("docs: revisione ADR-0030 D5 dopo review", True, None),
    ("scripts: hook commit-msg tracciato eseguibile", True, None),
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
