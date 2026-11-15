# ADR-0044: The topology ownership — the body is a generation

## Status

Proposed (2026-11-15) — the flip is the maintainer's own act. Questo
ADR è la proposta che apre la linea T1 (la caratterizzazione del
ciclo di vita della topologia); NESSUN morso di implementazione
parte prima dell'accettazione. Il morso m1 che lo consegna è
POCO CODICE E TUTTA MISURA: i quattro test di caratterizzazione
(`tests/unit/brep/test_t1_m1_topology_lifecycle.cpp`) pinnano il
comportamento ATTUALE — quello che è, non quello che si spera — e
questa proposta consuma le loro misure come base di evidenza. Se il
maintainer rovescia la scelta del contratto, i test restano: la
misura sopravvive alla decisione.

## Context — il muro che questa proposta dichiara, e quello che non tocca

La topologia esatta vive da G1 sotto una disciplina di possesso che
è sempre stata SCRITTA NEI COMMENTI e praticata nei call site, mai
stipulata come contratto: ADR-0021 §3 dichiara che i CoEdge
appartengono ai Loop e che `Edge::coedges_` è non-owning; la
finestra di validità dei puntatori radiali è un paragrafo nel
commento di `link_radial_edges`; la copia di un `BRepBody` porta
UNO solo membro shallow (le radiali) e ogni consumatore onesto la
chiude da sé (`link_radial_edges` sulla copia — il pattern che
`SceneNode`'s copy ctor e la catena del fillet praticano). Nessun
difetto è mai stato segnalato da questa lacuna; ma nessun
consumatore futuro può leggerne i confini, e la riga G5 ha appena
moltiplicato i produttori di rigenerazione (l'atto del ring
ri-finestra i fianchi, ADR-0043 D5). La campagna T1 apre con la
misura: questo ADR è il contratto che la misurazione sostiene.

Ciò che questa proposta NON tocca: la rappresentazione (le
strutture dati non si muovono), il piano degli indirizzi (ADR-0036
D1 lo ha congelato), i golden, la serializzazione (v0.8.0 porta
ricette e tavole, mai alberi di topologia — ADR-0007/0027). Il
contratto PROMUOVE a legge ciò che il codice già fa; dove il codice
lascia una porta aperta, il contratto la nomina e registra il morso
che la chiude.

## Decision

### D1. L'inventario della proprietà attuale (misurato, non dichiarato)

| Entità | Posseduta da | Riferita tramite | Alla copia |
|---|---|---|---|
| `Vertex` | `BRepBody::vertices` (vettore, per valore) | `VertexId` (indice) | profonda |
| `Edge` | `BRepBody::edges` (vettore, per valore) | `EdgeId` (indice) | profonda, TRANNE `coedges_` |
| `CoEdge` | `Loop::coedges_` (vettore, per valore) | `const CoEdge*` nudo, solo dentro `Edge::coedges_` (ADR-0021 §3) | profonda come membro del Loop |
| `Loop` | `Face::loops_` (vettore, per valore) | posizionale (`loops()[i]`) | profonda |
| `Face` | `Shell::faces_` (vettore, per valore) | `FaceId` (indice) | profonda |
| superficie | `shared_ptr` condivisa | `shared_ptr` | CONDIVISA (valori immutabili per convenzione) |
| `Solid`/`Shell` | membri pubblici di `BRepBody` | — | profonda |

I punti di copia esistenti: il copy ctor di `SceneNode` (ri-linka
le tre copie — fillet, ellipsoid, hybrid); la catena del fillet
(`cur = make_unique<BRepBody>(body)` + `link_radial_edges` nel
rebuild pass e in ogni `fillet_edge` interno); la copia dell'operando
nel subtract; il survival ledger, che riceve `const BRepBody*` come
INPUT di passo (il chiamante tiene vive le parti durante il walk).
La mutazione è sempre RICOSTRUZIONE INTERA: ogni edit (il raggio
del fillet compreso) costruisce un corpo nuovo e lo sostituisce
nell'`unique_ptr` del nodo; il corpo vecchio muore tutto insieme.

### D2. I quattro scenari, gli esiti misurati

La prova è nei test di caratterizzazione di m1 (170 asserzioni,
verdi su release; la colonna debug le giudica sotto ASan/UBSan alla
consegna):

1. **COPIA** — `BRepBody dst = *src`: le radiali di `dst` ALIASANO
   i Loop di `src` (il puntatore è lo stesso indirizzo; NESSUNA
   radial della copia punta alla copia stessa), le superfici sono
   condivise, gli id invariati. `link_radial_edges(dst)` chiude:
   ogni radial della copia risolve dentro la copia, gli invarianti
   strutturali passano. **La copia grezza NON è autonoma; la copia
   sanzionata è copia+relink.**
2. **MUTAZIONE** — `set_fillet_radius`: il corpo è UN'ALLOCAZIONE
   NUOVA (costruita mentre il vecchio è ancora vivo: gli indirizzi
   sono distinti PER COSTRUZIONE, deterministicamente); il NOME
   sopravvive e la feature ri-corre (`"reference_resolved"`); i
   conteggi sono invarianti nel raggio (la formula 8+2k/12+3k/6+k);
   la geometria è riderivata INTERA (una superficie nuova, il
   witness mantiene la vecchia). **Gli handle grezzi muoiono; i
   valori (nome, ricetta) sopravvivono.**
3. **DISTRUZIONE** — `remove_node`: il corpo muore con il nodo,
   tutto insieme (nessun superstite parziale, nessun zombie); la
   ricetta — un VALORE — ride-deriva la topologia IDENTICA
   (conteggi, χ, raggio: bit-deterministici). Il test NON
   dereferenzia mai un handle morto: quell'atto è l'UB che la
   colonna debug uccide FORTE (ASan: use-after-free, suite intera
   in abort), mentre il release la leggerebbe in silenzio. **La
   morte è totale; la derivazione è pura.**
4. **RIGENERAZIONE DEI LOOP** — l'atto del ring assorbe il fianco
   verticale (ADR-0043 D5): il fianco sopravvive PER NOME E STAMP
   (Absorbed, mai Born-sotto-falso), il suo loop è RIDERIVATO
   INTERO — stessa forma (1 loop, 4 coedge, tutti p-curved), edge
   set COMPLETAMENTE sostituito (il misurato {10, 8, 11, 9} →
   {45, 0, 42, 1}), gli indirizzi del loop pre-atto assenti dal
   corpo nuovo; i conteggi chiudono la formula (24/48/26, χ = 2,
   H = 0). **L'identità è un valore; il loop è materia di
   generazione.**

### D3. Il contratto: «il corpo è una generazione»

1. **IL POSSESSO È DEL CORPO, TOTALE ED ESCLUSIVO.** Il `BRepBody`
   possiede l'intero albero (Solid→Shell→Face→Loop→CoEdge, più
   Vertices ed Edges, per valore; le superfici condivise come
   VALORI immutabili). Nessun codice esterno possiede una parte;
   nessuna parte sopravvive al corpo.
2. **DENTRO la generazione: il piano degli indirizzi + la finestra
   chiusa.** I riferimenti intra-corpo sono gli indici
   (VertexId/EdgeId/FaceId — O(1), congelati da ADR-0036 D1) e i
   puntatori radiali `Edge::coedges_`, validi DALLA
   `link_radial_edges` ALLA successiva mutazione che aggiunge o
   rimuove facce/loop (la finestra chiusa di ADR-0021 §4, qui
   promossa a clausola).
3. **ATTRAVERSO le generazioni: SOLO NOMI.** Un riferimento che
   attraversa una copia, un edit o una rigenerazione è il NOME
   (ADR-0036 D1/D3) con la sua tavola — mai un indirizzo, mai un
   puntatore. La ricetta è la verità; il corpo è la cache di UNA
   generazione (ADR-0027 D3).
4. **LA COPIA SANCITA È UNA SOLA.** L'unico modo sanzionato di
   produrre una copia autonoma è copia + `link_radial_edges` —
   l'idioma che il copy ctor di `SceneNode` e la catena del fillet
   praticano da sempre. Il contratto lo NOMINA (`clone_body`,
   D7-m2) perché la porta smetta di essere un commento.
5. **I QUATTRO SCENARI COME ESITI GARANTITI** (i misurati di D2,
   elevati): la copia non è autonoma finché non è chiusa; la
   mutazione sostituisce il corpo intero e i nomi sopravvivono; la
   distruzione è totale e la derivazione pura; la rigenerazione
   conserva l'identità per nome e stamp e ridiriva la materia.

### D4. Le alternative scartate, col motivo

- **HANDLE STABILI GENERAZIONALI** (indici con contatore di
  generazione): risolve una stabilità cross-generazione che NESSUN
  consumatore ha — i nomi la danno già, strutturalmente, con la
  tavola e le quattro classi (ADR-0036 D3). Il costo lo paga il
  piano degli indirizzi che D1 ha congelato (ogni golden, ogni
  query posizionale), per un bisogno già servito. RIFIUTATA.
- **REGISTRY CENTRALE** (arena di `unique_ptr` con handle
  indiretti): il difetto misurato NON è il posto del possesso (i
  corpi già possiedono esclusivamente) ma la finestra dei puntatori
  radiali; un registry combatte la semantica di valore che la
  scena (unique_ptr nei nodi), l'undo/redo (deep clone) e il
  subtract (copia dell'operando) praticano, e lo fa pagare a ogni
  call site. L'arnese sbagliato per l'unica ferita. RIFIUTATA.
- **RADIAL SOLO-INDICE** (eliminare i puntatori, calcolare
  l'adiacenza on demand dagli indici): corretta e praticabile, ma
  tocca l'implementazione di OGNI query (`faces_around_edge`,
  `opposite_coedge`, `is_manifold`, le gate di K1 m5) per curare
  una rappresentazione che il contratto può lasciare DENTRO la
  finestra chiusa. REGISTRATA (D7) e prezzata al primo consumatore
  che la chieda, non scelta ora: la cura minima corretta.

### D5. L'impatto sulle porte esistenti — nessuna si rompe

- **ADR-0036 (naming)**: il contratto ADOTTA il nome-come-identità
  come propria gamba cross-generazione (clausola 3) — un
  rafforzamento, non un tocco. Il resolver, la tavola, le quattro
  classi: intatti.
- **Survival ledger (K1 m6)**: gli input `const BRepBody*` di ogni
  passo restano; il contratto ne documenta il dovere di vita (il
  chiamante tiene vive le parti durante il walk — come l'm6 fa). Il
  ledger legge valori (nomi, classi, digests): intatto.
- **Chain gate (K1 m5)**: conta e giudica corpi interi per conteggi
  e digest: intatta.
- **Serializzazione v0.8.0**: il formato porta RICETTE e TAVOLE,
  mai alberi di topologia (il corpo è cache, ADR-0007/0027) — il
  contratto non ha nulla sul filo. Il morso m2 (`clone_body`) non
  tocca né goldens né SSI né schema.
- **I test di m1 stessi**: se un domani una refactor (per esempio
  la D4-radial-only) cambierà un comportamento pinnato qui, il test
  rosso È la rete che dichiara il cambiamento — il pin si aggiorna
  nel morso che lo cambia, con la misura nuova accanto.

### D6. I muri — ciò che questo contratto NON garantisce

- **Nessuna storia di thread**: il rito resta single-threaded (il
  mondo di ADR-0009); il possesso non è protetto da race.
- **La finestra chiusa è documentata, non imposta da macchina**: un
  codice che aggiunga una faccia dopo la link e dereferenziali le
  radiali NON verrà fermato dal contratto (sarà fermato da ASan
  alla prima reallocazione — la rete dichiarata). L'imposizione è
  il morso registrato, non la promessa.
- **Le superfici condivise sono immutabili PER CONVENZIONE**: il
  tipo non lo impone; una mutazione const-violante di una superficie
  aliaserebbe attraverso le copie. La convenzione è il muro.
- **Il mondo faceted (il bridge)**: condivide le stesse strutture;
  i suoi percorsi (brep_to_sdf) non sono stati auditati
  dall'evidenza di m1 — il contratto li copre, la misura no.
- **La stabilità degli id di scena (undo/redo)**: è legge della
  scena (ADR-0036 D3/morse-4), non di questo contratto.

### D7. La stima dei morsi di implementazione (nulla parte prima del flip)

1. **T1 m2 — `clone_body`**: la copia sanzionata con un nome
   (copia + relink in UNA funzione), migrazione dei call site
   esistenti (SceneNode, la catena del fillet, il subtract) —
   meccanica, nessun golden si muove. La porta smette di essere un
   commento.
2. **T1 m3 — la finestra imposta** (se il maintainer la vuole): un
   timbro di generazione debug-only sul corpo, invalidato dalle
   mutazioni strutturali, letto da un assertion helper; il costo è
   misurato nel morso stesso, mai imposto al release.
3. **REGISTRATO, NON SCHEDULATO — la radial solo-indice** (D4): al
   primo consumatore che ne abbia bisogno; il suo morso dichiara e
   riprina i test di m1 con la misura nuova.

## Consequences

La topologia riceve il contratto che già pratica, con i confini
leggibili: la prossima riga G/T che produce rigenerazione troverà
la finestra, l'idioma e i quattro esiti GIÀ SCRITTI, e il
caratterizzatore di m1 come rete sotto ogni futuro cambio. La
misura resta la base: se il rovesciamento del maintainer sceglierà
un contratto diverso, sarà contro queste cifre, non contro il
vuoto.

## References

- ADR-0021 (la struttura dati e il §3/§4 che questo contratto
  promuove), ADR-0012 (radial-edge), ADR-0027 D3 (ricetta/corpo),
  ADR-0036 D1/D3 (nome/indirizzo, le quattro classi), ADR-0039
  (il walk del fillet), ADR-0043 D5/D6.3 (l'atto del ring, l'Absorbed).
- `tests/unit/brep/test_t1_m1_topology_lifecycle.cpp` — l'evidenza.
- `scripts/probe_t1_m1.cpp` — la sonda che misurò i numeri prima
  dei pin.
