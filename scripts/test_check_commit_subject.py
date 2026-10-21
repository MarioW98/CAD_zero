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
  3. la forma buona: l'estensione non azzanna i soggetti conformi —
     "fix(lint): ..." passa (la parentesi non matcha lo starter), la
     parola italiana DENTRO il soggetto non c'entra (la regola e' sullo
     START), e l'escape capitalizzato storico vale anche in italiano.

Uso:
  python3 scripts/test_check_commit_subject.py

Exit code: 0 se ogni caso esce come atteso, 1 altrimenti.
"""

import subprocess
import sys
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
    # case-sensitivity (la regex della traccia): l'escape capitalizzato
    # vale anche in italiano:
    ("Fix f-string Python 3.10", True, None),
    ("Correzione del calcolo AABB del cilindro", True, None),
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
