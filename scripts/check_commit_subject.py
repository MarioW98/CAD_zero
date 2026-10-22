#!/usr/bin/env python3
"""Commit subject lint — i soggetti devono essere descrittivi.

Regola (deriva dai 69 soggetti realmente usati nella storia del repo,
v. report nella commit che introduce questo script):

  un soggetto e' CONFORME se e solo se
    1. non e' un "tag puro" (U2, f.1, p1, 0027-....md, docs/adr, 5, ...):
       nemmeno un percorso di file e' una descrizione;
    2. NON inizia con uno starter vuoto (fix, fixing, various, check,
       end, init, upd, update, clean, rewrote, minor, majr — e gli
       equivalenti italiani correzione, sistemazione, modifica,
       aggiornamento, plurali inclusi) seguito da spazio, '/' o fine
       stringa;
    3. NON inizia con un'etichetta di progresso (intro, bozza,
       riepilogo, unita'/Unita' N) seguita da spazio, '/' o fine
       stringa: dice la FASE, non il contenuto — "intro kernel 2d" era
       15 caratteri esatti e passava col solo controllo lunghezza
       (run #90): il gap che questa regola chiude;
    4. è lungo almeno MIN_LENGTH caratteri (dopo lo strip).

  Il case e la forma Unicode non cambiano il difetto di un soggetto:
  "FIX della build" difetta esattamente come "fix della build", e la
  "à" precomposta vale la "A" + accento combinato (la forma
  decomposta di "UNITÀ 4 ..." scappava all'etichetta di progresso:
  la riproduzione che questa fix chiude). Le regole 1-3 giudicano
  quindi la forma NORMALIZZATA del soggetto — NFC, poi casefold (v.
  normalize_subject) — e i pattern sono scritti minuscoli. La regola
  4 no: la lunghezza si misura sulla stringa originale, perche' il
  casefold cambia la lunghezza di alcuni caratteri ("ß" -> "ss") e
  un minimo di caratteri non ha case. Conseguenza dichiarata: la
  lista storica pass/fail si aggiorna (l'escape capitalizzato "Fix
  f-string Python 3.10" ora e' uno starter vuoto come il suo gemello
  minuscolo), il giudizio sul difetto resta identico in ogni case.

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
import unicodedata

MIN_LENGTH = 15

# Tag puri: il soggetto e' SOLO un identificatore, zero contenuto.
# I pattern sono MINUSCOLI perche' il match avviene sulla forma
# normalizzata del soggetto (v. normalize_subject): "U2", "P12",
# "V1.2", "README.MD" e "DOCS/ADR" cadono come i loro gemelli
# minuscoli — la natura di un tag non cambia col case.
TAG_ONLY_PATTERNS = [
    (r"^u\d+$", "tag di unita' (U2, u3, ...)"),
    (r"^p\d+$", "prefisso numerico nudo (p1, P12)"),
    (r"^[a-z]?\d+[-.]\d+$", "versione numerica nuda (f.1, 2.3, V1.2)"),
    (r"^\d+([ .-]\d+)*$", "solo cifre (5, 2 3, 1-2)"),
    # subject = nome di file e basta ("0027-phase-f-....md")
    (r"^[a-z0-9_./\\-]+\.(md|txt|zip|py|cpp|hpp|h|cc|cxx|sh|json|yml|yaml|cmake|inl)$",
     "nome di file, non un soggetto"),
    # subject = percorso e basta ("docs/adr", "src/core/sdf"), con o
    # senza estensione: contiene '/' ed e' SOLO quello. Un soggetto che
    # nomina un percorso dentro una frase ("correzione ... stl/obj ecc")
    # ha spazi e parole: non matcha e resta giudicato dal resto delle
    # regole. Un percorso non e' una descrizione.
    (r"^[a-z0-9_.\-]+(/[a-z0-9_.\-]+)+/?$",
     "percorso di file, non una descrizione"),
]

# Starter vuoti: parole che nella storia compaiono solo in soggetti
# progress-marker. Ciascuna e' documentata nel report d'introduzione.
#
# Equivalenti italiani (dopo la run #93): "correzione" apre 9 soggetti
# storici, tutti marker — ma quello della run #93 aveva 10 caratteri ed
# e' caduto sulla LUNGHEZZA solo per caso: il marker deve cadere come
# marker, anche quando la lunghezza basterebbe ("correzione interfaccia
# 1": 24 caratteri, in storia). sistemazione, modifica e aggiornamento
# entrano per equivalenza con gli starter inglesi (fix, update, ...),
# con i plurali: nessun soggetto conforme della storia inizia con
# queste parole, in alcun caso. Il match e' sulla forma normalizzata
# (v. normalize_subject): il case non distingue piu' niente — "FIX",
# "Fix" e "fix" cadono nello stesso modo. La forma buona resta buona
# in ogni case ("fix(parser): ...", "FIX(lint): ...", "docs: ..."):
# i due punti non matchano lo starter, che vuole spazio, '/' o fine
# stringa. L'escape capitalizzato storico ("Fix f-string Python
# 3.10") muore qui: era un varco del case, non una forma buona.
BANNED_STARTERS = (
    r"^(fix|fixing|various|check|end|init|upd|update|clean|rewrote|minor|majr"
    r"|correzione|correzioni|sistemazione|sistemazioni"
    r"|modifica|modifiche|aggiornamento|aggiornamenti)"
    r"(\s|/|$)"
)

# Etichette di progresso: intro/bozza/riepilogo/unita'. Nella storia del
# repo compaiono SOLO in soggetti non conformi — "bozza sketch" (run #84),
# "Riepilogo:" (run #85), "Unità 2 sketch" (run #91) e "intro kernel 2d"
# (run #90: 15 caratteri esatti, PASSAVA col solo controllo lunghezza —
# il gap che questa regola chiude). Sono la versione in parole dei tag
# "U2"/"u3": dicono la fase del lavoro, non il contenuto. Stessa
# metodologia di derivazione degli starter vuoti (parole che in storia
# compaiono solo in soggetti non conformi). Come tutte le regole di
# testo, il match e' sulla forma normalizzata: "UNITÀ 4" maiuscolo —
# anche con la "à" scritta decomposta, che scappava — e' la stessa
# etichetta di "unità 4".
PROGRESS_LABELS = r"^(intro|bozza|riepilogo|unit[àa])(\s|/|$)"


def normalize_subject(subject: str) -> str:
    """La forma su cui le regole di TESTO giudicano: NFC, poi casefold.

    Il difetto di un soggetto non cambia col case ("FIX della build"
    difetta come "fix della build") e nemmeno con la forma Unicode
    dell'accento ("UNITÀ" precomposta = "A" + accento combinato: la
    forma decomposta scappava all'etichetta di progresso, chiusa
    qui). I pattern sono scritti minuscoli e NFC: il match avviene
    su questa forma.

    La LUNGHEZZA no: resta misurata sulla stringa originale, perche'
    il casefold cambia la lunghezza di alcuni caratteri ("ß" -> "ss")
    e un minimo di caratteri non ha case.
    """
    return unicodedata.normalize("NFC", subject).casefold()


def check_subject(subject: str) -> str | None:
    """Ritorna None se il soggetto e' conforme, altrimenti il motivo."""
    s = subject.strip()

    if not s:
        return "soggetto vuoto"

    # Prima la natura del soggetto (tag? starter vuoto?), poi la misura:
    # "U2" e' bocciato COME TAG, non come stringa corta. La natura si
    # giudica sulla forma normalizzata (il difetto non ha case); la
    # misura sulla stringa originale (v. normalize_subject).
    n = normalize_subject(s)

    for pattern, reason in TAG_ONLY_PATTERNS:
        if re.match(pattern, n):
            return reason

    if re.match(BANNED_STARTERS, n):
        return "starter vuoto (fix/various/correzione/modifica/...): il soggetto non dice niente"

    if re.match(PROGRESS_LABELS, n):
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
