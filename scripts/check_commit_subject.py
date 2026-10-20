#!/usr/bin/env python3
"""Commit subject lint — i soggetti devono essere descrittivi.

Regola (deriva dai 69 soggetti realmente usati nella storia del repo,
v. report nella commit che introduce questo script):

  un soggetto e' CONFORME se e solo se
    1. non e' un "tag puro" (U2, f.1, p1, 0027-....md, docs/adr, 5, ...):
       nemmeno un percorso di file e' una descrizione;
    2. NON inizia con uno starter vuoto (fix, fixing, various, check,
       end, init, upd, update, clean, rewrote, minor, majr) seguito
       da spazio, '/' o fine stringa (case-sensitive, come la regex
       della traccia: "Fix f-string Python 3.10" passa);
    3. NON inizia con un'etichetta di progresso (intro, bozza,
       riepilogo, unita'/Unita' N) seguita da spazio, '/' o fine
       stringa (case-insensitive): dice la FASE, non il contenuto —
       "intro kernel 2d" era 15 caratteri esatti e passava col solo
       controllo lunghezza (run #90): il gap che questa regola chiude;
    4. è lungo almeno MIN_LENGTH caratteri (dopo lo strip).

  Gli starter vuoti sono le parole che nella storia del repo
  compaiono SOLO in soggetti-progress-marker ("fix", "fix 3",
  "end E", "various fixes"...). La forma buona esiste ed e' quella
  conventional: "fix(parser): ...", "docs: ...", "test: ..." —
  i due punti non matchano lo starter (che richiede spazio o EOS),
  quindi NON sono bocciati.

  I commit di merge (2+ parent) sono saltati: il loro subject e'
  generato da git/GitHub, non scelto dall'autore.

Uso:
  check_commit_subject.py HEAD            # un singolo commit
  check_commit_subject.py A..B            # un range (es. i commit pushati)
  check_commit_subject.py --all           # tutta la storia (solo report)
  check_commit_subject.py --explain "..." # classifica una stringa
  check_commit_subject.py --commit-msg F  # il file del messaggio (hook git)

Exit code: 0 se tutti i commit esaminati passano, 1 altrimenti
(con --all l'exit code riflette la storia: e' un report, non un gate).
"""

import re
import subprocess
import sys

MIN_LENGTH = 15

# Tag puri: il soggetto e' SOLO un identificatore, zero contenuto.
TAG_ONLY_PATTERNS = [
    (r"^[uU]\d+$", "tag di unita' (U2, u3, ...)"),
    (r"^[pP]\d+$", "prefisso numerico nudo (p1, P12)"),
    (r"^[a-z]?\d+[-.]\d+$", "versione numerica nuda (f.1, 2.3, p0-p1)"),
    (r"^\d+([ .-]\d+)*$", "solo cifre (5, 2 3, 1-2)"),
    # subject = nome di file e basta ("0027-phase-f-....md")
    (r"^[A-Za-z0-9_./\\-]+\.(md|txt|zip|py|cpp|hpp|h|cc|cxx|sh|json|yml|yaml|cmake|inl)$",
     "nome di file, non un soggetto"),
    # subject = percorso e basta ("docs/adr", "src/core/sdf"), con o
    # senza estensione: contiene '/' ed e' SOLO quello. Un soggetto che
    # nomina un percorso dentro una frase ("correzione ... stl/obj ecc")
    # ha spazi e parole: non matcha e resta giudicato dal resto delle
    # regole. Un percorso non e' una descrizione.
    (r"^[A-Za-z0-9_.\-]+(/[A-Za-z0-9_.\-]+)+/?$",
     "percorso di file, non una descrizione"),
]

# Starter vuoti: parole che nella storia compaiono solo in soggetti
# progress-marker. Ciascuna e' documentata nel report d'introduzione.
BANNED_STARTERS = r"^(fix|fixing|various|check|end|init|upd|update|clean|rewrote|minor|majr)(\s|/|$)"

# Etichette di progresso: intro/bozza/riepilogo/unita'. Nella storia del
# repo compaiono SOLO in soggetti non conformi — "bozza sketch" (run #84),
# "Riepilogo:" (run #85), "Unità 2 sketch" (run #91) e "intro kernel 2d"
# (run #90: 15 caratteri esatti, PASSAVA col solo controllo lunghezza —
# il gap che questa regola chiude). Sono la versione in parole dei tag
# "U2"/"u3": dicono la fase del lavoro, non il contenuto. Stessa
# metodologia di derivazione degli starter vuoti (parole che in storia
# compaiono solo in soggetti non conformi). Case-insensitive perche' le
# occorrenze storiche includono "Unità" e "Riepilogo" capitalizzati e
# nessun soggetto conforme contiene queste parole in alcun caso.
PROGRESS_LABELS = r"(?i)^(intro|bozza|riepilogo|unit[àa])(\s|/|$)"


def check_subject(subject: str) -> str | None:
    """Ritorna None se il soggetto e' conforme, altrimenti il motivo."""
    s = subject.strip()

    if not s:
        return "soggetto vuoto"

    # Prima la natura del soggetto (tag? starter vuoto?), poi la misura:
    # "U2" e' bocciato COME TAG, non come stringa corta.
    for pattern, reason in TAG_ONLY_PATTERNS:
        if re.match(pattern, s):
            return reason

    if re.match(BANNED_STARTERS, s):
        return "starter vuoto (fix/various/check/end/...): il soggetto non dice niente"

    if re.match(PROGRESS_LABELS, s):
        return "etichetta di progresso (intro/bozza/riepilogo/unità N): la fase non è il contenuto"

    if len(s) < MIN_LENGTH:
        return f"troppo corto ({len(s)} < {MIN_LENGTH} caratteri)"

    return None


def run_git(*args: str) -> str:
    return subprocess.run(
        ["git", *args], capture_output=True, text=True, check=True
    ).stdout


def commit_subjects(spec: str, single: bool = True) -> list[tuple[str, str, bool]]:
    """Ritorna [(sha, subject, is_merge)] per la specifica data.

    "A..B" e' un range; qualunque altra cosa (SHA, HEAD) e' UN commit:
    git log <spec> seguirebbe tutta la storia di quell'oggetto, quindi
    di default il log si ferma a -1. Con single=False (modo --all) si
    segue invece tutta la storia raggiungibile, root compresa.
    """
    if ".." in spec:
        out = run_git("log", "--format=%H%x1f%s%x1f%P", spec)
    elif single:
        out = run_git("log", "-1", "--format=%H%x1f%s%x1f%P", spec)
    else:
        out = run_git("log", "--format=%H%x1f%s%x1f%P", spec)
    result = []
    for line in out.splitlines():
        if not line:
            continue
        sha, subject, parents = line.split("\x1f", 2)
        result.append((sha, subject, len(parents.split()) >= 2))
    return result


def main(argv: list[str]) -> int:
    if not argv:
        print(__doc__)
        print("uso: check_commit_subject.py HEAD | A..B | --all | --explain 'subject' | --commit-msg FILE")
        return 2

    if argv[0] == "--explain":
        subject = " ".join(argv[1:])
        if not subject:
            print("--explain richiede un soggetto (apici raccomandati)")
            return 2
        reason = check_subject(subject)
        print(f"PASS: {subject!r}" if reason is None else f"FAIL: {subject!r} — {reason}")
        return 0 if reason is None else 1

    if argv[0] == "--commit-msg":
        # uso da hook git (scripts/git-hooks/commit-msg): il file contiene
        # il messaggio, il soggetto e' la prima riga non vuota non-commento.
        # Stesso giudice, stessa funzione della CI: una sola regola, una
        # sola implementazione — l'hook non puo' divergere dal gate.
        if len(argv) != 2:
            print("--commit-msg richiede esattamente un percorso (uso da hook git)")
            return 2
        try:
            with open(argv[1], encoding="utf-8") as fh:
                lines = fh.read().splitlines()
        except OSError as exc:
            print(f"errore: lettura {argv[1]}: {exc}", file=sys.stderr)
            return 2
        subject = ""
        for line in lines:
            stripped = line.strip()
            if stripped and not stripped.startswith("#"):
                subject = stripped
                break
        reason = check_subject(subject)
        if reason is None:
            return 0
        print(f"FAIL: {subject!r} — {reason}")
        print("commit bloccato PRIMA della push: correggi il soggetto (--amend)")
        print("e riprova. Regola e derivazione: docstring di questo script")
        print("e CONTRIBUTING.md. La CI giudichera' comunque il range pushato.")
        return 1

    is_all = argv[0] == "--all"
    spec = "HEAD" if is_all else argv[0]

    try:
        commits = commit_subjects(spec, single=not is_all)
    except subprocess.CalledProcessError as exc:
        detail = (exc.stderr or "").strip() or f"git log {spec} fallito"
        print(f"errore: {detail}", file=sys.stderr)
        return 2

    if not commits:
        print(f"nessun commit per la specifica {spec!r}")
        return 0

    skipped = 0
    failures = []
    for sha, subject, is_merge in commits:
        if is_merge:
            skipped += 1
            continue
        reason = check_subject(subject)
        if reason is not None:
            failures.append((sha[:10], subject, reason))

    total = len(commits) - skipped
    print(f"commit esaminati: {total} (merge saltati: {skipped})")
    if failures:
        print(f"NON CONFORMI: {len(failures)}")
        for sha, subject, reason in failures:
            shown = subject if len(subject) <= 72 else subject[:69] + "..."
            print(f"  {sha}  FAIL  {shown!r}\n         motivo: {reason}")
    else:
        print("tutti conformi.")
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
