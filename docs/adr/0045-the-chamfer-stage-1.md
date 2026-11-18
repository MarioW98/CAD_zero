# ADR-0045: Il chamfer — stadio 1: il fianco piano e il miter sul cubo a dodici spigoli

## Status

Accepted (2026-11-18) — the flip is the maintainer's own act,
received in review and performed by the implementing agent on that
written instruction (the 0535ac6 precedent): the independent review
of the contract is PASS, the numbers of D6 verified by the reviewer
with derivations of their own (the miter, the counts 32/48/18, V(d)
= 8−12d²+6d³, the limit V(1) = 2 double-checked). Questo ADR è
la proposta che apre la linea G6; nessun morso di implementazione parte
prima dell'accettazione del maintainer. Il metodo è quello di casa
(ADR-0031 D5): l'ambito è ciò che i guide cases e la scala raggiungono
REALMENTE, ogni famiglia fuori rifiutata PER NOME. Il riferimento di
qualità resta la classe CGM/CATIA: lo smusso a setback costante è l'atto
esatto più antico del mestiere — il piano che taglia lo spigolo, il miter
che chiude l'angolo — mai una superficie procedurale spacciata per
esatta. La consegna di questa proposta è il documento SOLO: zero
righe .cpp/.hpp, zero golden mossi, zero file di serializzazione
toccati; ogni numero di D6 è derivato qui con la geometria scritta
visibile e verificato in forma chiusa su tutti gli otto angoli.

## Context — il muro che questo stadio abbatte, e quello che non tocca

La linea G5 ha chiuso il cubo raccordato per intero (ADR-0043, LANDED):
i dodici spigoli, le otto sfere d'angolo, la forma di Steiner, la
correttiva del 17/11 con il suo sweep oracolo. Ciò che resta aperto NON
è un muro del fillet: è la seconda feature che sostituisce invece di
sottrarre. Il chamfer è lo stesso gesto sullo stesso cubo — lo spigolo
muore, i piani adiacenti arretrano, un fianco nasce al suo posto — con
DUE differenze che questo ADR deriva prima del codice: il fianco è un
PIANO, non un cilindro (nessuna famiglia di superfici nuova: la
`PlaneSurface` esistente basta, verificata sull'albero — le sei facce
della factory, i kepts del subtract, il `PlaneReader` del fillet la
praticano da sempre); e l'angolo NON si chiude con una faccia (la sfera
del fillet) ma con un VERTICE, se la geometria lo ammette. La domanda
dell'angolo è la sola domanda nuova dello stadio, e la risposta è una
verifica, non un gusto: a setback uguali, i tre piani di smusso che
incontrano un angolo del cubo (x+y = 2−d, y+z = 2−d, x+z = 2−d
all'angolo (1,1,1)) si intersecano in UN SOLO PUNTO — il miter. La
verifica, scritta sotto (D2.3), CONFIRMA: l'angolo si chiude con un
vertice singolo, senza faccia triangolare; se avesse smentito, questo
ADR lo dichiarerebbe e sceglierebbe il facet, col motivo. Il macchinario
intorno è tutto provato dalla linea G5 e questo stadio lo RIUSA, non lo
reinventa: la walk per reference, la catena col gate a ogni passo, la
tavola, la grammatica del naming, la disciplina di ownership.

### Le letture che questa proposta consuma (tutte verificate sull'albero a `cb57522`)

- **ADR-0039** (il fillet, la classe 1): D1.1 la walk dello spigolo del
  box — l'aritmetica 10/15/7 con la tavola visibile, la formula della
  catena per il prisma di catena n (V = 2n+2, E = 3n+3, F = n+3), "the
  topology is n's alone; the geometry carries α"; D2 le bande tipizzate
  col rifiuto che nomina il bound; D4 la coda — IL CREASE MERGE ("Two
  adjacent fillets on the same corner: the flanks meet at the shared
  vertex in an untyped curve... the collision is the VERTEX-BLEND
  refusal"), la legge delle strisce sui PARALLELI (r₁ + r₂ < l'estensione
  della faccia), il collasso tangenziale al limite.
- **ADR-0043** (il fillet generale, stadio 1 — lo stampo di questa
  proposta): D3 la tesi "NESSUNA riga SSI nuova" coi contatti consumati
  per costruzione e la quartica mai consultata; D5 l'atto atomico
  d'angolo e l'ordine come derivazione (il ri-finestramento dei fianchi
  verticali, "replaced by the corner sphere's tangent arc", mai
  Born-sotto-falso); D6 i muri coi criteri di STOP; D7 i golden
  pre-pinnati con la forma chiusa e la quadratura; D8 il censimento che
  guadagna celle senza muovere righe.
- **tests/unit/brep/test_g5_m3_corner_oracle.cpp** — LA LEGGE che questo
  ADR prende come propria: "Do not use production construction helpers
  as test oracles" — oracoli a equazioni PROPRIE, mai dalla produzione;
  controprove che dottorano e devono essere rifiutate ("the contract
  discriminates, it does not rubber-stamp"); la verifica copre TUTTI gli
  istanti dello spazio dei casi (otto angoli × due ruoli × tre raggi —
  48 catene), mai un campione; i degeneri pinnati dal vivo, col nome.
- **ADR-0044** (ownership): il corpo è una generazione — attraverso le
  generazioni SOLO NOMI (D3.3); lo scenario misurato D2.4 È il
  precedente dell'atto di questo stadio: il fianco del ring assorbe il
  verticale PER NOME E STAMP (Absorbed, mai Born-sotto-falso), il loop
  riderrivato intero, l'edge set completamente sostituito.
- **ADR-0036 D1/D3** (naming): il nome funzione pura della ricetta; la
  grammatica `<feature>:<role>`; l'emendamento same-feature comprato
  dall'ADR-0039 ("or a strictly-earlier-born row of the SAME feature's
  construction order") — le tavole APPENDONO, gli indici frozen dai
  golden; le quattro classi del resolver come contratto di edit.
- **ADR-0041** (il protocollo di famiglia): D1.5 l'oracolo indipendente
  ("precision believed only when re-derived", la tolleranza dichiarata e
  la tolleranza onesta sul proprio condizionamento); D2 "the consumers
  eat the protocol, not the class" — aggiungere un consumatore significa
  che le celle del censimento approfondiscono le proprie prove, mai una
  riga nuova per una famiglia che già c'è.
- **Il codice, con inventario scritto**: `fillet.hpp` — la walk
  verticale (`fillet_edge`), il fianco del ring (`fillet_ring_edge`),
  l'atto d'angolo (`fillet_ring_edge_corner_act`) e l'atto batch
  (`fillet_ring_act`), con la lezione del 17/11 pinnata nel codice
  stesso: "which end is the corner's is a fact of the TOPOLOGY (the
  walk's start sits on the occupied vertex), never of the v-ordering" —
  i capi di una span si leggono dalla topologia, MAI dall'ordinamento
  delle coordinate; `naming.hpp` — le tavole dei ruoli, il
  `FeatureKind`, la disciplina dell'APPEND; `subtract.hpp` — le porte
  tipizzate (`SubtractNaming{feature, target, tool}`); `descent.hpp` —
  la tavola (`DescentRow`, la chiusura da entrambi i lati);
  `chain_gate.hpp` — i cinque predicati; `native.hpp` — la porta v0.8.0
  e il suo rito.

## Decision

### D1. Lo scope — il cubo a dodici spigoli, il setback dichiarato

**IN (i guide cases, la scala reale):**

1. **IL CUBO INTERAMENTE SMUSSATO** — il box [−1,1]³ della factory,
   tutti i 12 spigoli smussati IN CONTEMPORANEA a setback d: i 4
   verticali, gli 8 del ring; gli 8 angoli chiusi dal MITER (D4) —
   l'atto atomico che la convivenza degli smussi impone.
2. **SETBACK UNICO** — un solo d per tutti i 12 spigoli e gli 8 miter:
   la modalità d'oro, i conteggi e il volume in forma chiusa (D6).
3. **LA CAPACITÀ PER-SPIGOLO** — la derivazione onesta di ciò che la
   geometria esatta ammette: i setback possono differire LIBERAMENTE
   dentro ciascuna delle tre famiglie parallele (i 4 verticali, i 4 del
   ring X, i 4 del ring Y), soggetti alle condizioni di striscia sulle
   facce condivise (dᵢ + dⱼ < l'estensione della faccia); agli ANGOLI
   il miter esige l'uguaglianza dei tre setback che vi convergono
   (D5.4), e il grafo spigolo-angolo del cubo è CONNESSO: l'applicazione
   a tutti i 12 con miter esatti impone UN setback solo. La ricetta
   porta la MAPPA spigolo→setback (D7); l'ammissione la verifica per
   angolo e rifiuta col nome il misto.
4. **LE PARTIALI** — ogni sottoinsieme di spigoli che rispetti le
   condizioni di striscia (i 4 verticali con setback diversi; un ring
   solo; il cubo intero): tutte consumate dalla stessa macchina, la
   tavola che le conta derivata dal caso.

**OUT (la coda, per nome — mai in nota a piè di pagina):**

- **GLI SPIGOLI CURVI** — lo smusso sul bordo circolare del foro: il
  fianco sarebbe un CONO (la famiglia `ConeSurface` esiste, non
  trimmata, il suo flip atterrato col toro in ADR-0039 m3), il contatto
  piano-cono un'altra riga del mondo: ALTRA FAMIGLIA, la sede registrata
  (il precedente la classe rim del fillet, il criterio d'ingresso il
  primo consumatore reale con un rim da smussare). Nessuna data, nessuna
  promessa senza sede.
- **I SETBACK ASIMMETRICI** (d₁ ≠ d₂ sullo stesso spigolo): il piano di
  smusso obliquo alla bisettrice; la chiusura d'angolo degenera nella
  finestra asimmetrica — il cugino piatto del vertex blend variabile,
  fuori dal mondo esatto, rifiutato PER NOME (D5.4).
- **GLI ANGOLI GIÀ OCCUPATI DA FILLET** — l'angolo misto smusso+raccordo:
  il contatto bevel×fianco cilindrico è l'ELLISSE OBLIQUA (plane×cylinder
  obliquo: il rifiuto della tabella stessa), la collisione è la famiglia
  del CREASE MERGE, la sede ADR-0039 D4 (D5.3).
- **IL DIHEDRO GENERALE** (α ≠ π/2) — la stessa sede dello stadio 2 del
  fillet (la coda registrata di ADR-0039 D1.1 con il suo criterio
  d'ingresso: il consumatore reale non ortogonale); lo smusso ortogonale
  è la classe di questo stadio, il resto mai contrabbandato.

### D2. La tesi falsificabile — NESSUNA riga SSI nuova, il miter per forma chiusa

**La tesi centrale, derivata: il cubo interamente smussato si costruisce
senza consultare MAI una coppia non tipizzata e senza risolvere MAI
un'intersezione.** Ogni contatto della costruzione è uno di questi tre,
tutti LANDED o by-construction:

1. **IL FIANCO È UN PIANO** — la bisettrice a distanza dichiarata: il
   piano di smusso x+y = 2−d (per lo spigolo verticale tra right e back
   all'angolo (+,+)), la superficie `PlaneSurface` ESISTENTE (la
   famiglia della factory e del subtract — verificata sull'albero:
   NESSUNA famiglia di superfici nuova per lo stadio 1, nessuna finestra
   di rivoluzione, nessuna seam). Il fianco del fillet era un quarto di
   cilindro; il fianco dello smusso è un RETTANGOLO (o un esagono,
  all'angolo: D4) nel piano — la p-curve Line2 ovunque, il frame il
   piano's own, la chiralità mai in domanda.
2. **I CONTATTI SONO RETTE — LA RIGA LANDED plane×plane, CONSUMATA COME
   GRAMMATICA, MAI ESEGUITA COME SOLVE**: le RETTE DI SETBACK (bevel×
   piano adiacente: su ogni faccia la retta a distanza d dallo spigolo,
   gli estremi i punti di ritirata — AUTORATA per forma chiusa, la
   disciplina della classe v1 del fillet "ZERO intersection solves"
   verbatim) e le RETTE DI MITER (bevel×bevel: ogni estremo una forma
   chiusa dell'angolo e di d — D2.3). La riga plane×plane della tabella
   NON viene consultata dall'atto: le voci sono il VOCABOLARIO che i
   payload leggono, non solve che la feature esegue.
3. **IL MITER È UNA FORMA CHIUSA, MAI CONSULTATO COME INTERSEZIONE** —
   la verifica scritta: i tre piani di smusso che incontrano l'angolo
   (1,1,1) a setback uguali sono x+y = 2−d, y+z = 2−d, x+z = 2−d; le
   normali (1,1,0), (0,1,1), (1,0,1) hanno determinante 2 ≠ 0 (la
   concorrenza è un PUNTO, mai una retta: nessuno spigolo sopravvive
   all'angolo); la soluzione è x = y = z = 1−d/2 — **M = corner −
   (d/2)·(1,1,1)**, l'angolo arretrato di d/2 lungo la diagonale dello
   spazio, a distanza d/2 da ciascuno dei tre piani (DENTRO il setback
   d: il miter scava più a fondo della linea di ritirata). La forma è
   UNA per tutti gli otto angoli (le otto orientazioni il segno della
   diagonale — verificato simbolicamente su tutti e otto, la legge del
   correttiva: le otto orientazioni una forma chiusa sola). I punti di
   ritirata sono a loro volta forme chiuse (il punto della faccia top:
   corner − d·(1,1,0); e le permutazioni); le tre rette di miter
   corrono da M ai tre punti di ritirata, ognuna condivisa da esattamente
   i due bevel i cui piani la contengono — ogni capo una forma chiusa,
   NESSUNA intersezione risolta.

**IL CRITERIO DI STOP, allegato alla tesi**: se durante
l'implementazione servisse UNA riga SSI nuova, UNA famiglia di superfici
nuova, o UNA qualunque intersezione risolta (il miter, una retta di
setback, una retta di miter consultati come solve), la campagna SI FERMA
e questo ADR si emenda col difetto registrato: il difetto sarebbe
dell'implementazione, non della tesi. Il claim è verificabile dal diff
del primo morso: toccare `ssi.hpp`/`ssi.cpp` per questo stadio è un
difetto, non una scelta.

### D3. La grammatica del naming — `che#F:bevel[...]`, le tavole, le reference

Sullo stampo del fillet (ADR-0039 D3), con la disciplina dell'APPEND
(ADR-0043 D2.5: le righe nuove accodano, gli indici del fillet frozen
dai suoi golden):

- **IL TIPO**: `FeatureKind::Chamfer`, token **`che`** — la kind table
  guadagna la riga (il protocollo delle madri di ADR-0036 D6: l'estensione
  vive nelle tavole, le parole delle madri stanno).
- **IL RUOLO DEL FIANCO**: **`che#F:bevel[<face-A> × <face-B>]`** — la
  forma Relational, gli operandi le due facce adiacenti dello spigolo
  riferito NELL'ORDINE DEL CO-EDGE DELLO SPIGOLO (lo stampo di
  `fil#F:flank[...]` verbatim, la stessa determinismo input-order-
  independent). Il guide case: `che#2:bevel[box#1:back ×
  box#1:right]` (il primo smusso sul box#1:vertical2 della factory).
  Il token è `bevel` — il vocabolo della letteratura (Hoffmann) per il
  fianco piatto dello smusso, distinto dal `flank` cilindrico del fillet
  come il `corner` della sfera fu distinto: UN ruolo nuovo per una
  geometria nuova, mai un overload di un ruolo che significa altro.
- **LE RETTE — il pattern `isect` esistente, il livello ASSENTE** (la
  tangenza È il livello, la forma delle elisioni m3/m4): le rette di
  setback **`che#F:isect[<la faccia che le porta> × che#F:bevel[...]]`**
  (lo stampo dei fili tangenti del fillet); le rette di miter
  **`che#F:isect[<bevel-A> × <bevel-B>]`** — gli operandi renderizzati
  sotto le tavole dei PROPRI creatori (la regola per-creator del
  morse-5): il miter ring-ring dello STESSO atto viaggia sull'estensione
  same-feature che l'ADR-0039 comprò (entrambi gli operandi righe nate
  PRIMA nel construction order dell'atto); il miter vertical-ring
  riferisce il bevel della feature PIÙ VECCHIA (strictly older — legale
  dal primo giorno). **ADR-0036 resta NON EMENDATO**: la grammatica
  chiude con le parole che il fillet già comprò.
- **LA TAVOLA**: `chamfer_descent` sullo stampo di `fillet_descent` —
  le righe Born/Vanished/Absorbed/Kept con la chiusura da entrambi i
  lati; le cause verbatim: "replaced by the bevel" (gli spigoli morti),
  **"replaced by the miter line"** (i segmenti di capitello morti
  nell'atto — l'analogo letterale di "replaced by the corner sphere's
  tangent arc").
- **LA REFERENCE**: la porta della stringa — la edge-reference della
  ricetta matchata INTERA contro i nomi degli spigoli del corpo (la
  forma propria di `fillet_reference_find`, il rimedio nel payload,
  mai un mute not found).
- **I VERTICI NASCONO SENZA NOME** (la coda D4 di ADR-0036: nessun
  consumatore oggi) — i punti di ritirata e i miter portano le
  POSIZIONI come identità nei golden, la stessa disciplina dei punti di
  tangenza della sfera.

### D4. L'atto d'angolo — il miter come atto atomico, sullo stampo D5 di ADR-0043

#### D4.1 La decomposizione e l'ordine (la derivazione)

1. **IL PASSAGGIO VERTICALE** (la classe di oggi, una famiglia sotto):
   i 4 smussi verticali, sequenziali o in un atto (libertà
   dell'implementazione, la tavola conta gli stessi numeri), GATED a
   ogni passo (i cinque predicati): ogni passo È la walk di ADR-0039
   D1.1 — lo spigolo muore (−1 spigolo, −2 vertici), le due facce
   adiacenti arretrano alle rette di setback (absorbed), i due
   capitelli perdono l'angolo (absorbed, il loop ri-autorato col
   SEGMENTO di smusso — una RETTA, mai un arco), nascono 4 spigoli e 4
   vertici e 1 faccia: 10/15/7 al primo, la formula 8+2k/12+3k/6+k fino
   a 16/24/10. I 4 verticali non condividono angoli: NESSUN contatto
   nuovo.
2. **L'ATTO DEL RING E DEI MITER, INDIVISIBILE**: gli 8 bevel del ring
   e gli 8 miter in UN SOLO atto. L'indivisibilità, DERIVATA per questa
   geometria con l'onestà che la casa deve al confronto col fillet:
   per il fillet la via sequenziale era BLOCCATA dalla parete (la
   quartica, mai tipizzabile); per lo smusso la via sequenziale è
   TIPIZZATA (le rette bevel×bevel sono la riga landed) — ma la
   chiusura è una concorrenza a TRE: il miter nasce dai TRE piani
   congiuntamente, NESSUNO stato a due bevel lo contiene, e ogni ordine
   sequenziale ri-finestra facce nate prima nella stessa ricetta (il
   terzo bevel all'angolo ri-taglia i primi due: la retta di miter del
   primo incontro muore, sostituita da quella che chiude il trio).
   L'atto rende la chiusura
   atomica: i bevel del ring nascono GIÀ RIFINITI (entrambi i capi
   mitrati), i miter e i punti di ritirata nascono, gli smussi verticali
   si RI-FINISTRANO in place. La tavola dell'atto è UNA, le otto
   orientazioni una forma chiusa sola.

#### D4.2 Il ri-finestramento del bevel verticale — mai Born-sotto-falso

Il bevel verticale (nato nel passaggio) SOPRAVVIVE per nome e stamp:
la sua finestra rettangolare (la sezione d × l'estensione) diventa
l'ESAGONO — le due rette di setback RI-SPANATE (gli estremi sostituiti
dai punti di ritirata: la sostituzione la regola propria del fillet,
"un capo morto si sostituisce col vertice della ritirata del piano che
il bordo confina"), i due segmenti di capitello MORTI con la causa
verbatim "replaced by the miter line", le quattro rette di miter nate
ai suoi due angoli. La riga della tavola legge **Absorbed** — lo
scenario misurato D2.4 di ADR-0044 è il precedente esatto: mai una
morte e una nascita sotto lo stesso nome (il Born-sotto-falso che il
test rifiuta), mai lo stamp perso.

**EMENDAMENTO (G6 m3, 2026-11-18 — l'atto a UN ANGOLO, D8.3):** la
descrizione sopra è la composizione a DUE angoli (l'atto del ring
completo). L'atto a UN angolo — il morso che per primo materializza
questo paragrafo — ha le sue FORME PROPRIE, derivate prima del codice
e qui scritte: il bevel verticale si ri-finestra a **PENTAGONO** (UN
solo segmento di capitello morto all'angolo consumato, le due rette
di miter che vi convergono, la causa verbatim invariata); i bevel del
ring nascono **PENTAGONI** — il piano non taperizza, e all'estremo
lontano lo stesso piano taglia l'angolo NON mitrato: un SEGMENTO
CAPO-LONTANO sulla faccia laterale remota, il vertice remoto morto e
i suoi due spigoli (la verticale e la ring edge) re-spanati ai piedi
remoti. L'esagono è la composizione additiva: il secondo atto
ri-finestra il pentagono, e i capi lontani muoiono "replaced by the
miter line" negli atti partner. I conteggi dell'atto a un angolo dal
10/15/7 della catena k=1: **14/21/9, χ = 2** (4 vertici e 3 spigoli
morti, 8 vertici, 9 spigoli e 2 facce nati), il volume
**V = 8 − 3d² + (3/4)d³** (l'inclusione-esclusione all'angolo: tre
prismi pieni 3d², meno tre sovrapposizioni a coppie 3·(d³/3), più la
tripla d³/4 col tetra-split det/6). I criteri di STOP di D2 restano
onerati: nessuna riga SSI, nessuna famiglia di superfici nuova (il
miter un VERTICE, mai una faccia).

**LANDING (G6 m4, 2026-11-18):** la composizione a due angoli è
landed per intero — `chamfer_ring_act` (l'atto indivisibile dallo
stato a quattro verticali 16/24/10) e `chamfer_twelve_edges` (la
porta completa: i quattro passaggi gated e l'atto in UNA
transazione, gli intermedi interni alla porta, UNA feature sola per
i dodici bevel), gli esagoni interi, i conteggi 32/48/18, il volume
V(d) = 8 − 12d² + 6d³ con l'oracolo indipendente a codice zero
condiviso — il pentagolo a un angolo qui sopra il passo additivo che
lo raggiunge.

#### D4.3 L'aritmetica dell'atto (la tavola contata, la chiusura da entrambi i lati)

Dal 16/24/10 del passaggio verticale: **muoiono 16 vertici** (le punte
dei capitelli) **e 16 spigoli** (i 8 spigoli del ring del cubo + gli 8
segmenti di capitello); **nascono 32 vertici** (i 24 punti di ritirata
— quattro per faccia ritirata — e gli 8 miter), **40 spigoli** (le 16
rette di setback del ring e le 24 rette di miter — tre per angolo) **e
8 facce** (i bevel del ring). Lo stato finale: **V = 32, E = 48, F =
18, H = 0, χ = 2** — Δχ = +16 −24 +8 = 0, la walk χ-flat come ogni
walk della casa. Il censimento dei co-edge: 12 bevel × 6 + 6 piani × 4
= 96 incidenze / 2 = 48 ✓ (ogni spigolo esattamente due co-edge
opposti — l'identità della output gate); ogni punto di ritirata:
3 facce (un piano e due bevel) e 3 spigoli; ogni miter: 3 facce (i tre
bevel) e 3 spigoli (le tre rette di miter) — MANIFOLD a ogni vertice,
verificato.

#### D4.4 La lezione del 17/11 — REQUISITO del morso, non opzione

La correttiva G5 m3 (commit `ede4224`) pagò il difetto che questo ADR
scrive come legge: il ri-finestramento leggeva il capo dell'angolo
come il minimo della v-span (vero solo con l'angolo in alto) — la
finestra usciva la span MORTA dell'angolo a ogni angolo inferiore, 24
catene su 48 rosse alla prima run, il volume 7.84781 contro la forma
chiusa 7.69902. LA CURE, citata: "the corner's end is a fact of the
walk's start on EITHER junction, v_occ or v_occ2; the far end
unchanged, the direction preserved — the eight orientations one closed
form". Questo ADR esige: **ogni capo di ogni span di ogni faccia nata
o ri-finestrata si legge dalla TOPOLOGIA dell'angolo (la walk sulle
giunzioni, il vertice occupato e il suo partner), MAI
dall'ordinamento delle coordinate**; e la SWEEP SUGLI OTTO ANGOLI (×
entrambi i ruoli del ring × i setback in banda) è PARTE DEL MORSO
d'angolo — l'oracolo a zero codice condiviso, le controprove che
dottorano, i degeneri pinnati dal vivo (la legge del m3, verbatim).

### D5. I muri — ogni rifiuto col suo nome

1. **IL SETBACK FUORI BANDA** — `chamfer_setback_exceeds_face`, il
   bound NEL payload (la faccia, la sua portata, l'ε_L): l'ammissione
   dello spigolo singolo legge la portata delle facce dalla riga di
   setback (d < la portata dallo spigolo − ε_L; per il cubo: d < 2−ε_L
   — la lettura propria di `fillet_radius_exceeds_face`, il 1.99
   ammesso contro il 2.0 rifiutato). **LA BANDA DELLO STADIO, DERIVATA:
   d < 1**: su ogni faccia le due strisce degli smussi opposti devono
   non incontrarsi (2d < 2); le regioni dei prismi di spigoli
   non-adiacenti restano vuote sse d < 1 (la derivazione: la tripla
   {x+y > 2−d, x+z > 2−d, x−z > 2−d} dà 2x > 4−2d, e con x ≤ 1 è vuota
   sse d ≤ 1); a d = 1 LE SEI FACCE MUIONO A PUNTI e i dodici bevel
   diventano i rombi — IL COLLASSO TANGENZIALE, la forma chiusa lo VEDE
   (D6: V(1) = 2, il rombododecaedro) e l'ammissione lo RIFIUTA (il
   fillet's r=1 sphere precedent: the body is not the limit). Il miter
   non è mai il vincolo vincolante: M sta a 1−d/2, dentro il cubo per
   ogni d < 2 — la striscia lega prima. **LA NOTA DEL LANDING
   (G6 m4):** alla PORTA COMPLETA il collasso a d = 1 è preso UN MURO
   PRIMA dalla legge delle strisce stessa — la macchina m1 legge la
   portata CORRENTE della faccia ri-finestrata (il primo smusso l'ha
   stretta a 2−d), e la banda di faccia della SECONDA walk rifiuta
   esattamente a d = 1 con il bound nel payload; il rifiuto NOMINATO
   della banda di stadio vive alla porta dell'atto (dove la lettura
   doppia — la portata delle facce contro il bound dello stadio — è
   il payload). Entrambi i muri onesti, entrambi nominati.
2. **LO SPIGOLO CURVO** — `chamfer_out_of_class_pair`, la famiglia
   NEL payload: lo smusso sul bordo del foro avrebbe per fianco un
   CONO (la famiglia esiste, il suo writer no: la sede è la classe rim
   dello smusso, il primo consumatore reale il criterio — il precedente
   la m3 del fillet). L'ammissione della classe rifiuta per FIRMA
   STRUTTURALE (le adiacenze non planari), mai con un test di
   geometria a run time: la firma è la garanzia, la produzione non può
   produrre un portatore fuori classe.
3. **L'ANGOLO OCCUPATO DA UN FILLET** — l'angolo misto: il bevel piano
   e il fianco cilindrico si incontrano lungo l'ELLISSE OBLIQUA — il
   rifiuto è della TABELLA stessa (plane×cylinder obliquo: "not
   typifiable in the G1 vocabulary... Refused with the angle"). La
   collisione è la famiglia del CREASE MERGE, la sede ADR-0039 D4
   (la coda col consumatore nominato: il confine tangenziale dei
   fillet paralleli); il rifiuto dell'atto è alle FIRME DI CLASSE (le
   facce curve del corpo rifiutano PER FAMIGLIA — le stesse parole
   della catena inversa del fillet), la sede nel payload.
4. **IL MISMATCH DI SETBACK SULLA RICETTA MULTI-SPIGOLO** —
   `chamfer_corner_setback_mismatch`, i valori NEL payload (lo stampo
   di `fillet_corner_radii_mismatch`): la legge del miter, DERIVATA —
   i tre piani a setback disuguali si intersecano ancora in un punto
   (le normali indipendenti), ma il punto ESCE dall'ottante dell'angolo
   salvo le disuguaglianze triangolari (dᵢ ≤ dⱼ+dₖ), e la chiusura
   diventa la finestra asimmetrica — il vertex blend variabile PIATTO,
   fuori dal mondo esatto; la chiusura pulita a vertice singolo con le
   finestre esagonali è la geometria del setback UGUALE. Il rimedio nel
   payload (uguagliare all'angolo, o applicare per famiglia); e la
   legge del grafo connesso impone UN d per la modalità a 12 spigoli
   (la legge dei raggi di ADR-0043 D2.4, un parametro sopra).

### D6. I golden — tutti a forma chiusa, con la controprova indipendente

Ogni numero qui sotto è derivato con la geometria visibile (la
concorrenza simbolica sugli otto angoli, l'inclusione-esclusione, il
tetra-split, la quadratura sui pezzi in parametrizzazione propria);
al morso dell'oracolo il test RI-DERIVA con equazioni proprie (la
legge del m3: mai dalla produzione; le controprove che dottorano; la
sweep su TUTTI gli istanti dello spazio dei casi, mai un campione).

1. **LO SPIGOLO VERTICALE SINGOLO, d = 0.5**: i conteggi **10/15/7,
   χ = 2, H = 0** — GLI STESSI del fillet k=1 (la coerenza strutturale
   è essa stessa un pin: la walk è la stessa aritmetica, i 4 spigoli
   nati sono tutti RETTE dove il fillet aveva 2 rette e 2 archi); il
   volume: il solido rimosso è il PRISMA TRIANGOLARE RETTO di cateti d
   e altezza 2 (la sezione d²/2, il braccio 2): **V = 8 − d² = 7.75**
   (31/4, esatto); i limiti: d → 0 → 8 ✓, d → la banda (D5.1).
2. **LA CATENA A DUE SPIGOLI NON ADIACENTI** (lo stampo k=2 del
   fillet, la m6ter): **12/18/8, χ = 2**, V = 8 − 2d² = 7.5 a
   d = 0.5; gli stati della formula 10/15/7 → 12/18/8 → 14/21/9 →
   16/24/10, i primi due i golden esistenti del fillet IMMOVI (la
   clausola di additività: ogni golden landed resta al suo posto).
3. **I DODICI SPIGOLI CON IL MITER**: lo stato finale **V = 32,
   E = 48, F = 18, H = 0, χ = 2** (l'aritmetica dell'atto, D4.3; il
   censimento dei co-edge 96/2); **il volume rimosso per angolo in
   forma chiusa** — la regione dei tre semi-spazi nell'ottante
   dell'angolo (u,v,w ∈ [0,1], l'insieme rimosso {(u+v ≤ d) ∨ (u+w ≤ d)
   ∨ (v+w ≤ d)}): per inclusione-esclusione, 3·(d²/2) − 3·(d³/3) +
   d³/4 — i tre mezzi-cunei (la sezione triangolare d²/2 per il braccio
   unitario), le tre intersezioni a coppie (il quadrato di lato d−u per
   u: d³/3), la tripla (il poliedro a cinque vertici — l'origine, i
   tre punti d sugli assi e il miter d/2 — scomposto in TRE tetraedri,
   det/6: d³/4): **R_c = (3/4)·d²·(2−d)**; la somma sugli otto angoli
   (congruenti per simmetria del cubo): 12d² − 6d³; **V(d) = 8 −
   12d² + 6d³**, il golden numerico **V(0.5) = 5.75** (23/4, esatto).
   **LA CONTROPROVA A QUADRATURA INDIPENDENTE** (la strada numerica,
   l'idioma road-3 dell'oracolo di Steiner landed: i pezzi nelle
   PROPRIE parametrizzazioni — il cuneo, la coppia e la tripla nelle
   sezioni esatte, i termini polinomiali esatti, MAI un indicatore su
   una griglia sforacchiata): **2·10⁶ campioni midpoint**, la barra
   dichiarata **2·10⁻⁶** relativa; gli accordi misurati: 1.74·10⁻¹³ a
   d = 0.5 e 4.33·10⁻¹⁴ a d = 1/3 (il secondo istante). LA NOTA DI
   CONDIZIONAMENTO, onesta (ADR-0041 D1.5): il campionamento nudo
   dell'indicatore sul box dell'angolo erra O(h) — misurato 5.17·10⁻³
   a 126³ — i bordi PIATTI non hanno cancellazione; le
   parametrizzazioni dei pezzi sono il protocollo, come per il fillet
   furono il disco in polare e l'ottante in sferiche.
   **I LIMITI**: d → 0 → 8 ✓; **d → 1 → 2 — IL ROMBODODECAEDRO**: le
   sei facce muoiono nei punti (±1,0,0)..., i dodici bevel diventano i
   rombi, gli otto miter siedono a (±1/2,±1/2,±1/2) — 14 vertici, 12
   facce, 24 spigoli, χ = 2 ✓; la forma classica (16/9)√3·a³ con
   a = √3/2 dà 2 ESATTO — il collasso VISTO dalla forma chiusa e
   RIFIUTATO in ammissione (D5.1).
4. **I PIN DI COERENZA**: E = 48 GLI STESSI dello stato a dodici
   spigoli del fillet (24/48/26) — la composizione diversa (24 setback
   + 24 miter contro 24 tangenti + 24 archi), il conto uguale: la
   coerenza strutturale essa stessa un pin; il singolo 10/15/7 = il
   k=1 del fillet; le due forme chiuse affiancate a 0.5 — il fillet
   per apertura morfologica 4+11π/12 = 6.879793265790644, lo smusso
   23/4 = 5.75: il bevel piatto rimuove PIÙ del rullo a parità di
   parametro (il triangolo d²/2 > il cuneo r²(1−π/4) per unità di
   lunghezza), e i due collassi differiscono (la sfera inscritta
   (4/3)π contro il rombododecaedro 2) — lo stesso cubo, due geometrie,
   ogni numero derivato.

### D7. Le porte — la ricetta è un tipo nuovo, il formato decide, il censimento guadagna un consumatore

1. **LA RICETTA È UN TIPO NUOVO**: `ChamferRecipe` — la edge-reference
   come STRINGA (la forma della grammatica, character for character) e
   il setback come campo TIPATO suo proprio (d > 0, finito): due campi
   nominali, la disciplina m4 verbatim (MAI un indirizzo, MAI un dump
   di struct); e la forma multi (la MAPPA spigolo→setback, l'ordine
   autorato, l'unicità della reference rifiutata ad alta voce a ogni
   porta) sullo stampo di `MultiFilletRecipe`. LA LEGGE DEL SETBACK è
   letta ALLE PORTE che portano il corpo (la creazione, il pass 2 del
   loader, la porta di salvataggio col nome del mismatch e i valori)
   — MAI dal kernel: l'ammissione del kernel resta l'ultima linea,
   le sue parole immutate.
2. **LA PORTA DEL FORMATO: v0.9.0 — DECISO, col motivo**: un nodo
   nuovo è la classe ESATTA di cambiamento che bumpò v0.7→v0.8 (il
   `fillet_multi`, "THE CAPABILITY BUMP'S SECOND RITE"); il campo
   versione è il CONTRATTO DEL LETTORE (un file che dichiara 0.8.0
   deve essere interamente leggibile da OGNI lettore 0.8.0 — un nodo
   chamfer dentro un file 0.8.0-dichiarante sarebbe la menzogna che
   la porta esiste per impedire); ospitare il tipo nuovo in v0.8.0
   RIFIUTATO per questo. IL RITO: la legge di additività (OGNI BYTE DI
   v0.8 VIAGGIA INTERO — nessun file v0.8 porta un nodo chamfer:
   niente da perdere), il gate anacronistico (il payload rifiuta un
   nodo chamfer in ogni file che dichiara < 0.9 — la bugia
   hand-edited, la forma propria del fillet), il passo 0.8.0→0.9.0
   della catena migrator (no-op), la companion declaration
   (ADR-0007/0027), MAI due bump lo stesso giorno — il numero
   arbitrato al tempo del morso che lo consegna (il precedente
   v0.5/v0.6).
3. **IL CENSIMENTO GUADAGNA UN CONSUMATORE** — la riga PLANE: la
   cella WRITER guadagna il sito di nascita del bevel (il quarto
   genere dopo la factory, i capitelli dell'estrusione e i kepts del
   subtract: un piano NATO da una feature — l'operazione che lo
   authoring non produceva ancora); le altre celle APPROFONDISCONO le
   prove esistenti (la massa: il flusso poligonale al teorema della
   divergenza, la macchina planare di Green già nell'albero, esatta;
   le righe SSI: plane×plane consumata come grammatica; i degeneri:
   le disposizioni coplanari già pinnate) — NESSUNA cella indietro,
   NESSUNA riga nuova di famiglia (la legge di ADR-0041: il censimento
   conta le FAMIGLIE correttamente supportate, e le famiglie sono le
   stesse; "the consumers eat the protocol, not the class").
4. **PICKER E BINDINGS SONO MORSI SUCCESSIVI, NON DI QUESTO ADR**: la
   porta UI sullo stampo m5 del fillet (le righe del picker DICHIARANO
   prima di ogni Apply — i 12 in classe, i curvi FUORI con la famiglia
   nominata, gli angoli occupati rifiutati con la sede; il report dice
   tutto — i conteggi col termine H, la provenienza, la colonna dei
   nomi, il volume in forma chiusa).

### D8. La stima dei morsi — ciascuno col suo rischio dichiarato

Le unità di lavoro, contate alla scala della casa (i morsi del fillet
generale furono 6–7; la chiusura K1 1):

1. **LO SPIGOLO SINGOLO** (la walk, il golden 10/15/7 + 7.75, la
   tavola ~35 righe sullo stampo proprio del fillet, le bande
   d'ammissione col rifiuto che nomina il bound, i bindings
   raggiungibili da Python) — UN MORSO. Rischio: IL PIÙ BASSO dello
   stadio — la walk del fillet una famiglia SOTTO (il piano è più
   semplice del cilindro: nessuna seam, nessuna periodicità, le
   p-curve Line2 ovunque, nessun arco).
2. **LA CATENA K=2** (lo stampo m6ter: 12/18/8, il volume 7.5, i nomi
   annidati, il timbro tenuto) — MEZZO MORSO (la stessa macchina
   rieseguita). Rischio: solo l'annidamento dei nomi.
3. **L'ATTO D'ANGOLO CON LA SUA SWEEP SUGLI OTTO ANGOLI** (l'atto a un
   angolo: le forme chiuse, il ri-finestramento del bevel verticale,
   la tavola dell'atto con le cause verbatim; l'ORACOLO a zero codice
   condiviso: gli otto angoli × entrambi i ruoli del ring × i setback
   in banda, le controprove che dottorano, i degeneri pinnati dal
   vivo — la sweep REQUISITO, D4.4) — DUE MORSI. Rischio: IL CUORE E
   IL RISCHIO dello stadio — il ri-finestramento (i capi di span
   letti dalla topologia, mai dall'ordinamento: la correttiva del
   17/11 è il prezzo d'ingresso già pagato una volta dalla linea G5,
   e la sua storia è il tagliando del rischio).
4. **I DODICI SPIGOLI** (l'atto del ring e dei miter: 32/48/18,
   l'aritmetica dell'atto, la forma del volume, l'oracolo a
   quadratura, il survival ledger, i digest) — UN MORSO. Rischio: la
   tavola dell'atto (16 morti e 40 nascite intrecciate — la chiusura
   da entrambi i lati al primo tentativo, la forma propria del muro
   D6.2 del fillet: se una riga non torna, STOP, il rapporto nomina
   la riga).
5. **LA RICETTA E LA PORTA V0.9.0** (il ChamferRecipe, la forma multi,
   il rito del bump con la companion e il migrator, i golden del
   round-trip — i golden v0.8 IMMOVI) — UN MORSO. Rischio:
   l'arbitrazione del numero (mai due bump lo stesso giorno).
6. **LA PORTA UI** (il picker che dichiara, il report, il pannello, i
   bindings) — MEZZO MORSO; e IL CLICK-TEST l'uscita: il cubo con
   tutti i dodici spigoli smussati e gli angoli che si chiudono a
   punta, VISIBILE nel viewport e MISURABILE (il pannello dice 5.75 a
   d = 0.5, il report dice 32/48/18) — la frase del maintainer che
   chiude lo stadio: "riesco a smussare tutti gli spigoli del cubo,
   e gli angoli si chiudono a punta."

TOTALE STIMATO: 5–6 morsi. I CRITERI DI STOP vincolano ognuno (D2 il
criterio della tesi; D4.4 la sweep; D5 i muri); il primo morso è lo
spigolo singolo (nessun rischio, la fondazione); il terzo è il cuore.

## Consequences

- La seconda feature che sostituisce ha il suo contratto: il fianco
  piano, il miter in forma chiusa, i conteggi e i volumi pinnati PRIMA
  del codice con la derivazione visibile, i rifiuti col nome e col
  bound, la clausola di additività che tiene ogni golden landed
  byte-identico.
- La tabella SSI non cambia di una riga (D2 — il claim verificabile
  dal diff del primo morso); nessuna famiglia di superfici nuova: il
  bevel è un `PlaneSurface`, la finestra l'unica cosa che nasce.
- La domanda dell'angolo è risolta dalla DERIVAZIONE, non dalla
  preferenza: il miter — un vertice, nessuna faccia — verificato per
  concorrenza simbolica sugli otto angoli; e il giorno in cui un
  consumatore chiederà la chiusura asimmetrica, il rifiuto ha già il
  nome (`chamfer_corner_setback_mismatch`) e la sede.
- La lezione del 17/11 è contrattuale: la sweep sugli otto angoli è
  parte del morso d'angolo (il requisito, non l'opzione), e i capi di
  span si leggono dalla topologia — scritto nell'ADR prima che nel
  codice.
- La grammatica guadagna UN tipo (`che`) e i suoi ruoli (le tavole
  APPENDONO, gli indici del fillet frozen); ADR-0036 resta non
  emendato — l'estensione same-feature che il fillet comprò copre i
  fili del miter.
- Il formato guadagna la porta v0.9.0 (il rito del capability bump,
  la legge di additività, il gate anacronistico) — il numero
  arbitrato al suo tempo; il censimento guadagna un consumatore: la
  riga PLANE approfondisce le prove, nessuna riga si muove.
- Lo stadio successivo ha le sue sedi registrate (lo smusso conico
  sul rim; il dihedro generale con lo stadio 2 del fillet; l'angolo
  misto col crease merge di ADR-0039 D4) — nessuna data, nessuna
  promessa senza sede.

## References

- Records di casa: ADR-0039 (D1.1 la walk e la formula della catena;
  D2 le bande tipizzate; D4 la coda — crease merge, vertex blend, la
  legge delle strisce), ADR-0043 (lo stampo: D3 la tesi SSI, D5 l'atto
  atomico e il ri-finestramento, D6 i muri, D7 i golden con la
  quadratura, D8 il censimento, D9 i morsi), ADR-0044 (D2.4 lo
  scenario misurato del ri-finestramento Absorbed; D3 i nomi attraverso
  le generazioni), ADR-0036 (D1 la grammatica e l'emendamento
  same-feature; D3 le quattro classi), ADR-0041 (D1.5 l'oracolo
  indipendente e la nota di condizionamento; D2 i consumatori del
  protocollo), ADR-0035 (i mondi tipizzati che le bande leggono),
  ADR-0033 (D2 il criterio both-domain), ADR-0023 (le convenzioni
  seam — citate per contrasto: il bevel non ne ha bisogno),
  ADR-0007/0027 (le porte del formato, la companion declaration).
- Il codice: `fillet.hpp` (le walk, l'atto d'angolo e la cure del
  17/11 — "a fact of the TOPOLOGY... never of the v-ordering"),
  `naming.hpp` (le tavole dei ruoli, la disciplina dell'APPEND),
  `descent.hpp` (la tavola), `chain_gate.hpp` (i cinque predicati),
  `native.hpp` (la porta v0.8.0 e il suo rito — il modello della
  decisione v0.9.0).
- I test: `test_g5_m3_corner_oracle.cpp` (la legge dell'oracolo che
  questo ADR prende come propria), `test_g2_u3_m6ter_the_chain.cpp`
  (lo stampo della catena), `test_g5_m4_ring_act.cpp` (l'idioma
  road-3 della quadratura: i pezzi nelle proprie parametrizzazioni).
- La letteratura: Hoffmann 1989 (i capitoli sul blending — lo smusso
  come limite planare della famiglia del rolling ball, il vertex
  blend che questo stadio rifiuta per nome); Mäntylä 1988 (gli
  operatori di Euler su cui l'aritmetica dei conteggi sta); e le
  forme classiche che sono i test, mai le implementazioni: il prisma
  d², la regione d'angolo (3/4)d²(2−d), il rombododecaedro
  (16/9)√3a³.
