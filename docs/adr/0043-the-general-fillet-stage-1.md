# ADR-0043: Il fillet generale — stadio 1: tutti gli angoli del cubo

## Status

Accepted (2026-11-14) — the flip is the maintainer's own act. Questo ADR è la
proposta che apre la linea G5; nessun morso di implementazione parte
prima dell'accettazione del maintainer. Il metodo è quello di casa
(ADR-0031 D5): l'ambito è ciò che i guide cases e la scala raggiungono
REALMENTE, ogni famiglia fuori rifiutata PER NOME. Il riferimento di
qualità è la classe CGM/CATIA: il raccordo costante come apertura
morfologica del solido (la strada di Steiner), il vertice come atto
atomico, mai una superficie procedurale spacciata per esatta.

## Context — il muro che questo stadio abbatte, e quello che non tocca

Il click-test del 2026-11-08 (ADR-0039, verbalizzato alla riga m6 del
registro): "Riesco a fare i fillet solo per gli spigoli paralleli
all'asse blu (credo sia l'asse z), ossia solo a quelli 'vertical'." —
"solo vertical" è il class wall che l'ADR-0039 D1.1 inchioda: la classe
v1 prende lo spigolo VERTICALE, il fianco la famiglia CylinderZ, e il
picker DICHIARA prima di ogni Apply (i 4 verticali IN CLASSE, gli 8
spigoli del ring FUORI con la famiglia nominata, i riferimenti morti
VANISHED con il killer). Il dihedro generale e i fianchi a asse X/Y
sono REGISTRATI per il loro primo consumatore (D4, la coda per nome) —
questo ADR è quel consumatore, e solo per il caso che la geometria
esatta davvero raggiunge: il cubo.

Ciò che la campagna K1 ha chiuso il 2026-11-14 (la suite di
consumazione reale) lascia il debito nominato: IL MURO VERTICAL-ONLY,
con il suo sede. La derivazione della parete verticale è già
parametrica nell'angolo (ADR-0039 D1.1: "the topology is n's alone;
the geometry carries α") ma non nell'ASSE: il fianco nasce sempre
CylinderZ perché la classe v1 ammette solo lo spigolo verticale. Il
ring del cubo — 8 spigoli orizzontali, dihedri a 90° tra facce di
piano esattamente come i verticali — chiede solo la stessa matematica
UN ASSE SOTTO: CylinderX e CylinderY. Il dihedro NON ortogonale
rimane fuori: quello è lo stadio 2, e la sua porta è registrata qui
sotto (D8), non promessa senza sede.

### Le letture che questa proposta consuma (tutte verificate sull'albero a `e1db075`)

- **ADR-0039 D1.1/D4**: la classe box-edge (10/15/7, la forma chiusa
  del cuneo abc − ℓ·r²(1−π/4)), la formula della catena per il prisma
  di catena n (V = 2n+2, E = 3n+3, F = n+3), il muro vertical-only,
  la coda: IL CREASE MERGE — "Two adjacent fillets on the same corner:
  the flanks meet at the shared vertex in an untyped curve
  (cylinder×cylinder perpendicular is the Steinmetz quartic — out of
  the table in BOTH domains, ADR-0033 D2's own derivation;
  cylinder×torus likewise) — the collision is the VERTEX-BLEND
  refusal, `fillet_corner_is_vertex_blend`"; i fillet su spigoli
  PARALLELI di una stessa faccia sono indipendenti e in classe FINCHÉ
  la striscia intermedia sopravvive (r₁ + r₂ < l'estensione della
  faccia, entrambe le bande derivate); la scomparsa della striscia al
  limite è il collasso tangenziale (rifiutato, il tie-break golden) e
  la sua morte esatta il vertex blend.
- **La correttiva-ter di G2 e la catena m6ter (K1 m6, membro 4)**: il
  fillet consuma lo stato corrente — la catena box-family
  8/12/6 → 10/15/7 → 12/18/8, cioè V = 8+2k, E = 12+3k, F = 6+k per k
  fillet verticali sequenziali (verificata a k = 1 e k = 2 sui golden
  dell'albero), i nomi del secondo fianco nati sotto fil#3 con la
  grammatica annidata (`fil#3:flank[box#1:front × box#1:left]`,
  `fil#3:isect[box#1:front × fil#3:flank[...]]`), il timbro tenuto che
  sopravvive al primo anello e muore al secondo — il ledger legge.
- **Le tabelle SSI (G2 u1, il kernel)**: le righe LANDED che questo
  stadio consuma — plane×sphere col punto tangente (il degenere
  tipizzato), cylinder×sphere coassiale col CERCHIO TANGENTE (il
  degenere tipizzato: cerchi a ±√(R²−r²), che a R = r degenera nel
  cerchio di tangenza), cylinder×cylinder PARALLELI a raggi uguali
  (LinePair: le due rette nel piano medio, d < 2r); I MURI — la coppia
  cilindri NON parallela è la quartica irriducibile il cui membro di
  Steinmetz non è tipizzato in NESSUN dominio; la coppia parallela a
  raggi DISUGUALI è rifiutata al cancello della riga registrata CON LA
  DISPOSIZIONE CORRETTA SUL RECORD: il caso trasversale |r₁−r₂| < d <
  r₁+r₂ è DUE RETTE TIPIZZATE nel piano sfalsato x = (d²+r₁²−r₂²)/(2d)
  — ammessa dal criterio, fuori dalla riga registrata fino all'unità
  che la consuma; mai chiamata "untypable", che non è; la PROVENANCE
  GATE — la tabella esatta consulta solo facce ESATTE, un operando
  approssimato è rifiutato col rimedio nominato.
- **ADR-0041 (il protocollo di famiglia)**: la famiglia nuova entra
  attraverso le SEI RIGHE (la parametrizzazione con la sua seam, la
  regola del frame puntuale col pin di chiralità, la massa in forma
  chiusa o quadratura dichiarata, il vocabolario delle bande con lo
  Jacobiano della famiglia come derivazione nominata, le disposizioni
  dei degeneri, la riga di serializzazione), il censimento a sette
  colonne conta le famiglie CORRETTAMENTE supportate — mai le feature.
- **I golden del fillet esistente**: 10/15/7 (un verticale, r = 0.5,
  il volume 7.5 + π/8), 14/20/9 (il rim cieco, 6.775059758517764),
  la forma V_r del cuneo — tutti IMMOVI (la clausola di additività
  D1.4 di ADR-0039: ogni golden esistente resta al suo posto).

## Decision

### D1. L'ambito dello stadio 1 — il cubo, tutto il cubo, nient'altro che il cubo

**IN (i guide cases, la scala reale):**

1. **IL CUBO INTERAMENTE RACCORDATO** — il box [−1,1]³ della factory,
   tutti i 12 spigoli raccordati IN CONTEMPORANEA:
   - i 4 spigoli verticali (Z-paralleli) — la classe esistente, i
     fianchi CylinderZ;
   - gli 8 spigoli del ring (4 X-paralleli, 4 Y-paralleli) — LA
     CLASSE NUOVA di questo stadio, i fianchi CylinderX/CylinderY;
   - gli 8 angoli (i vertici a tre facce) — LE SFERE D'ANGOLO, l'atto
     atomico che la convivenza dei raccordi impone (D5), ogni
     contatto tipizzato (D4).
2. **RAGGIO UNICO** — un solo r per tutti i 12 spigoli e le 8 sfere:
   la modalità d'oro, i conteggi e il volume in forma chiusa (D7).
3. **RAGGI DIVERSI PER SPIGOLO** — la derivazione onesta di ciò che la
   geometria esatta ammette (D2.4): i raggi possono differire LIBERAMENTE
   dentro ciascuna delle tre famiglie parallele (i 4 verticali — in
   classe già oggi, per la famiglia parallela di D4; i 4 del ring X; i
   4 del ring Y), soggetti alle condizioni di striscia sulle facce
   condivise (rᵢ + rⱼ < l'estensione della faccia, entrambe le bande
   derivate); agli ANGOLI la sfera esige l'uguaglianza dei tre raggi
   che vi convergono (D5.2 — la legge della sfera, derivata), e il
   grafo spigolo-angolo del cubo è CONNESSO: l'applicazione a tutti
   i 12 con sfere d'angolo esatte impone UN raggio solo. La
   convivenza di raggi diversi allo stesso angolo è il VERTEX BLEND
   variabile — fuori dal mondo esatto, rifiutata PER NOME
   (`fillet_corner_radii_mismatch`, D6), con il rimedio nominato
   (uguagliare all'angolo, o applicare per famiglia). Questa è la
   lettura onesta del brief: la CAPACITÀ per-spigolo è il vocabolario
   della ricetta (la mappa spigolo→raggio esiste, serializzata, D3.5);
   la FISICA dell'angolo esatto ne vincola i valori.
4. **LE PARTIALI** — ogni sottoinsieme di spigoli che rispetti le
   condizioni di striscia e la legge degli angoli (i 4 verticali con
   raggi diversi — già in classe; un ring solo; il cubo intero a
   raggio unico): tutte consumate dalla stessa macchina, la tavola
   che le conta derivata dal caso (D7.3).

**OUT (la coda, per nome — mai in nota a piè di pagina):**

- **IL DIHEDRO GENERALE ARBITRARIO (α ≠ π/2) — LO STADIO 2**, registrato
  qui con il suo CRITERIO DI INGRESSO (D8): un consumatore reale con un
  giunto non ortogonale (un prisma autorevole inclinato) e la
  derivazione della finestra del capitello (π−α) e della distanza
  tangente t = r·cot(α/2) già scritte in ADR-0039 D1.1. Nessuna data,
  nessuna promessa senza sede.
- **IL VERTEX BLEND VARIABILE** (raggi disuguali allo stesso angolo) —
  la famiglia procedurale, il mondo approssimato di G3: la sua porta è
  la provenance gate, la sua sede è la coda di D4/ADR-0039.
- **IL COLlasso TANGENZIALE** (la striscia che muore: rᵢ + rⱼ =
  l'estensione della faccia; r = 1 il caso estremo del cubo) —
  rifiutato col tie-break golden, la morte esatta è il vertex blend.
- **LA FAMIGLIA CONCAVA, IL RAGGIO VARIABILE LUNGO LO SPIGOLO, IL
  FILLET DI FACCIA** — le code di ADR-0039 D4, intatte, non toccate da
  questo stadio.

### D2. La derivazione dei fianchi — CylinderX e CylinderY: la stessa matematica, un asse sotto

#### D2.1 Le superfici

La famiglia `CylinderZSurface` esistente è la superficie di rivoluzione
(raggio r, asse Z, base z₀): il piano (u, v) = (azimut θ, quota) col
frame destro {û = cos θ · x̂ + sin θ · ŷ, v̂ = ẑ, n̂ = û} e la seam
all'azimut dichiarato. **L'estensione è la stessa superficie con l'asse
X o Y**: nessuna nuova matematica, un cambio di frame:

- `CylinderXSurface(r, x₀, y_c, z_c)`: asse x̂, il cerchio di base nel
  piano x = x₀ centrato (y_c, z_c); il frame {û = (0, cos θ, sin θ),
  v̂ = x̂, n̂ = û × v̂ destro per costruzione — il costruttore deriva il
  secondo asse dal prodotto, la chiralità NON è un knob (la legge del
  piano lineare, una famiglia sopra)}.
- `CylinderYSurface(r, y₀, x_c, z_c)`: la stessa costruzione sull'asse ŷ.

La seam: l'azimut θ = 0 dichiarato per costruttore (il riferimento
l'asse +Z per CylinderX, +X per CylinderY — i piani di riferimento
delle factory, congelati dai golden), la slit-rectangle pattern di
ADR-0023 per la faccia-finestra esattamente come la Z.

#### D2.2 Il fianco del ring — la finestra θ × la striscia

Lo spigolo del ring (per esempio top∩right: l'orizzontale Y-parallela
tra le facce box#1:top (z = +1) e box#1:right (x = +1)) porta la palla
di raggio r tangente a entrambi i piani: il centro sulla bisettrice a
distanza r·cot(π/4) = r dallo spigolo — la retta (x, 1−r, 1−r), e
l'involuppo è **CylinderYSurface(r, ·, 1−r, 1−r)**, il fianco la
finestra di quartiere θ ∈ [θ₀, θ₀+π/2] (i punti di tangenza: sulla
top il cerchio tocca a z = 1 quando θ punta +ẑ; sulla right a x = 1)
× la lunghezza y ∈ [−(1−r), (1−r)] dopo i verticali (D5). La
costruzione è CHIUSA in ogni quantità: le rette di tangenza piazzate
esattamente (sulla top la retta x = 1−r; sulla right la retta z = 1−r),
zero intersezioni risolte — la stessa disciplina della classe v1 ("the
ball's center rides the bisector... ZERO intersection solves"), un asse
sotto.

#### D2.3 I conteggi — la formula della catena box-family, verificata

Per k fillet VERTICALI sequenziali (la correttiva-ter/m6ter, i golden
a k = 1, 2): **V = 8+2k, E = 12+3k, F = 6+k** — lo walk di ADR-0039
D1.1 ripetuto (lo spigolo muore: −1 spigolo, −2 vertici; le due facce
adiacenti si rifinestrano (assorbite); i due capitelli perdono
l'angolo (assorbiti, il loop ri-autorato); nascono 4 spigoli, 4
vertici, 1 faccia). La stessa formula vale per i fillet del ring
applicati al box pulito k per k (gli spigoli del ring sono verticali
NEL LORO frame — la stessa topologia, il piano di simmetria ruotato) —
MA gli spigoli del ring ADIACENTI all'angolo collidono (D5), e per
questo l'ordine di consumo è quello di D5, non il sequenziale nudo.

**Lo stato finale del cubo intero (raggio unico) — la derivazione
diretta, il censimento dei co-edge visibile:**

- **Le facce**: 6 piani rifinestrati (ognuno il rettangolo
  [−1+r, 1−r]² nel proprio piano — D5.3 ne deriva i confini) + 12
  fianchi cilindrici (4 Z + 4 X + 4 Y) + 8 ottanti di sfera (le sfere
  d'angolo) = **F = 26**.
- **Gli spigoli**: 24 rette di tangenza (ogni fianco ne porta 2, una
  per faccia adiacente: piano↔fianco, ogni spigolo esattamente due
  co-edge opposti) + 24 archi di cerchio tangente (ogni fianco ne
  porta 2, uno per estremità: fianco↔sfera, il cerchio di tangenza
  coassiale, l'arco di quartiere) = **E = 48**.
- **I vertici**: 3 per angolo (i punti di tangenza della sfera sui
  tre piani — per esempio all'angolo (1,1,1): (1−r, 1−r, 1) sulla top,
  (1, 1−r, 1−r) sulla right, (1−r, 1, 1−r) sulla back) × 8 angoli =
  **V = 24**.
- **La chiusura**: χ = V − E + F − H = 24 − 48 + 26 − 0 = **2** ✓ (il
  corpo è una sfera topologica, genere 0 — il cube eroso e riaperto);
  il censimento dei co-edge: 6 piani × 4 + 12 fianchi × 4 + 8 sfere ×
  3 = 96 incidenze / 2 = 48 ✓ (ogni spigolo esattamente due co-edge
  opposti — l'identità della output gate).
- **H = 0**: nessun loop interno (le finestre semplicemente connesse).

#### D2.4 I raggi diversi — la legge degli angoli, derivata

Le tre famiglie parallele del cubo sono tra loro a coppie
NON-ADIACENTI DENTRO la famiglia (i 4 verticali non condividono
angoli; i 4 del ring X nemmeno; i 4 del ring Y nemmeno): raggi
liberamente diversi DENTRO ogni famiglia, soggetti alle condizioni di
striscia (sulla faccia condivisa tra due spigoli paralleli: rᵢ + rⱼ <
l'estensione — la striscia che sopravvive). ALL'ANGOLO la sfera
(D5.2) esige r_verticale = r_ringX = r_ringY: una sfera tangente ai
tre piani mutuamente ortogonali ha il centro a (1−r, 1−r, 1−r) e il
suo raggio È la distanza da ciascun piano — tre raggi distinti non
hanno sfera. Il grafo spigolo-angolo del cubo (12 spigoli, 8 angoli,
ogni angolo congiunge 3 spigoli) è connesso: l'uguaglianza propagata
per ogni angolo forza l'unicità — la modalità a 12 spigoli con sfere
esatte È il raggio unico, detto dalla geometria e non da un gusto.
La ricetta porta la MAPPA spigolo→raggio (D3.5); l'ammissione la
verifica per angolo e rifiuta col nome il misto.

#### D2.5 Il naming — la grammatica estesa di un ruolo solo

I fianchi del ring: la STESSA grammatica dei verticali (le due facce
adiacenti permutate nell'ordine del picker): `fil#F:flank[box#1:top ×
box#1:right]` — nessuna estensione necessaria. Le rette di tangenza e
gli archi: il pattern isect esistente, l'annidamento compreso
(`fil#F:isect[box#1:top × fil#F:flank[...]]`, il m6ter lo pratica già).
**L'estensione è UN ruolo nuovo**: la sfera d'angolo,
`fil#F:corner[box#1:top × box#1:right × box#1:back]` — le tre facce
adiacenti nell'ordine del picker (la prima quella del primo spigolo
del riferimento), il creatore il fillet che l'ha partorita, la
provenienza dichiarata nelle righe della tavola (Born, il padre
l'atto). I punti e gli archi di tangenza della sfera:
`fil#F:isect[fil#F:corner[...] × fil#F:flank[...]]` per gli archi (il
cerchio coassiale), il punto di tangenza al piano un VERTICE nato
dell'atto (i vertici senza nome per la coda D4 di ADR-0036, la
posizione il ruolo — la disciplina esistente, non un nuovo vocabolario).

### D3. Le SSI necessarie — la tesi dello stadio: NESSUNA riga nuova

**La tesi centrale, derivata**: il cubo interamente raccordato si
costruisce senza consultare MAI una coppia non tipizzata. Ogni
contatto della costruzione è uno di questi tre, tutti LANDED o
by-construction:

1. **Fianco×piano (la retta di tangenza)** — AUTORATA PER COSTRUZIONE,
   la disciplina della classe v1 ("the contact circles placed
   exactly... the SSI entries' own typing... never run as solves: the
   table's surface unchanged"): la retta è la forma chiusa degli
   operandi (il piano a distanza r dal centro del cerchio), la
   p-curva Line2 u = const sul dominio del cilindro, Line2 sul piano.
   La riga plane×cylinder della tabella NON viene consultata
   dall'atto (come oggi per il vertical-only).
2. **Sfera×cilindro COASSIALE (il cerchio di tangenza)** — la riga
   LANDED cylinder×sphere coassiale, il suo degenere tipizzato al
   limite R = r: il cerchio di tangenza nel piano perpendicolare
   all'asse per il centro della sfera. La sfera d'angolo e il fianco
   adiacente sono COASSIALI PER COSTRUZIONE (il centro della sfera
   STA sull'asse del cilindro: (1−r, 1−r, 1−r) ∈ {(1−r, 1−r, z)} ✓),
   il cerchio piazzato esattamente, la p-curva Circle2 sul dominio
   della sfera (il parallelo) e Line2 u = const sul cilindro. NESSUN
   solve, la chiusura è l'aritmetica.
3. **Sfera×piano (il punto di tangenza)** — la riga LANDED plane×sphere
   al suo degenere tipizzato (il punto al piede della perpendicolare).

**Il muro della quartica, DICHARATO e mai toccato**: il fianco del
vertical e il fianco del ring allo stesso angolo sono cilindri ad assi
PERPENDICOLARI — la loro intersezione completa È la quartica di
Steinmetz, fuori dalla tabella in ENTRAMBI i domini (ADR-0033 D2, la
disposizione confermata dal wall test dell'albero). Lo stadio 1 non
risolve, non approssima, non aggiunge la riga: la regione d'angolo è
riempita dalla SFERA (D5), e i fianchi sono AUTORATI GIA' RIFINITI
dalla sfera (l'arco del cerchio coassiale è il bordo della finestra,
non il risultato di un'intersezione). La coppia
cilindro×cilindro-perpendicolari NON VIENE MAI CONSULTATA — questa è
l'affermazione falsificabile dello stadio, e il suo criterio di STOP
(D6): se un'implementazione si trova a dover risolvere
cilindro×cilindro perpendicolari, l'implementazione è SBAGLIATA, non
il muro.

**La riga dei raggi disuguali paralleli — la sede registrata, non
consumata**: la coppia parallela a raggi disuguali col caso trasversale
|r₁−r₂| < d < r₁+r₂ (due rette tipizzate nel piano sfalsato, la
disposizione corretta già sul record della tabella) NON serve allo
stadio 1: i fianchi paralleli con la striscia viva NON si toccano, e la
striscia morta è il collasso tangenziale (rifiutato). La riga resta
fuori "fino all'unità che la consuma" — la sua sede è il vertex blend
variabile / la fusione del crease, la coda di ADR-0039 D4. Nessuna
promessa qui.

**La provenance gate, EREDITATA**: ogni operando del fillet deve
essere ESATTO (la faccia approssimata rifiutata col rimedio) — la
stessa porta dello SSI esatto, nessuna deroga per lo stadio.

### D4 (in D3 sopra, la sede dei muri) — ciò che la tabella NON riceve

Riassunto della disciplina SSI dello stadio: ZERO righe nuove, ZERO
cambi alle righe esistenti, i tre contatti consumati (tangenza
fianco×piano by construction; cerchio coassiale sfera×fianco, il
degenere landed; punto tangente sfera×piano, il degenere landed), la
quartica mai consultata, la riga dei disuguali paralleli con la sede
nominata e vuota. La superficie della tabella è la stessa di oggi —
il claim è verificabile dal diff del primo morso: toccare
`ssi.hpp`/`ssi.cpp` per questo stadio è un difetto, non una scelta.

### D5. L'applicazione simultanea — l'angolo è l'atto, l'ordine è la derivazione

#### D5.1 Perché il sequenziale nudo muore all'angolo (la derivazione dell'ordine)

Applicare i 12 fillet uno alla volta in un ordine qualunque incontra
l'angolo PRIMA della sfera: il secondo fianco che arriva a un angolo
già occupato da un primo fianco deve rifinirsi CONTRO il primo —
cilindro×cilindro perpendicolari, LA QUARTICA — a meno che la sfera
non sia già lì. La sfera, d'altronde, è incompleta senza i SUOI tre
fianchi (il suo ottante è delimitato dai tre cerchi di tangenza —
senza un fianco non c'è l'arco che chiude il patch). La conclusione è
una catena di dipendenze circolare attorno all'angolo, e la sua
soluzione è la decomposizione atomica:

1. **L'ATTO VERTICALE** (la classe di oggi): i 4 fillet verticali,
   sequenziali o in un atto solo (libertà dell'implementazione, la
   tavola conta gli stessi numeri), GATED a ogni passo (K1 m5): ogni
   passo è il walk di ADR-0039 D1.1 su un box i cui spigoli verticali
   sopravvissuti non condividono angoli tra loro — NESSUN contatto
   nuovo, i capitelli ri-autorati con gli archi di quartiere (la
   sweep π−α = π/2), lo stato finale del passaggio 16/24/10 (la
   formula 8+2k con k = 4, verificata dai golden a k = 1, 2 — il
   morso la pinna anche a k = 3, 4).
2. **L'ATTO DEL RING E DEGLI ANGOLI, INDIVISIBILE**: gli 8 fianchi
   del ring e le 8 sfere d'angolo in UN SOLO atto — nessuno stato
   intermedio della decomposizione è chiuso (un fianco del ring senza
   le sue due sfere alle estremità ha i bordi Non tipizzati; una sfera
   senza i suoi fianchi ha il patch non delimitato; mezzo atto è un
   corpo che la gate RIFIUTA — e questo È la definizione di
   indivisibilità). L'atto nasce tutto insieme: le 8 sfere ai centri
   (1−r, 1−r, 1−r) + le 7 permutazioni di segno, gli 8 ottanti (i
   patch sferici delimitati dai tre archi di quartiere dei cerchi
   coassiali — D5.2), i fianchi del ring autorati con la finestra
   θ-quarto × la lunghezza [−(1−r), (1−r)] e i bordi già rifiniti
   dagli archi delle sfere, le facce di piano ri-autorate alle rette
   di tangenza, gli archi dei capitelli dei verticali MORTI (i
   quarti di cerchio sui piani top/bottom sono interamente consumati
   dalle strisce del ring — la loro morte è una riga VANISHED della
   tavola col killer nominato), gli stessi fianchi verticali
   RI-FINISTRATI (l'arco del capitello sostituito dall'arco del
   cerchio coassiale della sfera — l'edit in-place del m6, il ledger
   legge la sopravvivenza).

Lo stato dopo l'atto: **24/48/26, H = 0, χ = 2** (D2.3), la gate verde
(i cinque predicati, la tavola dell'atto contata: Born gli 8 fianchi +
8 sfere + i loro bordi; Vanished gli 8 spigoli del ring + i 16 archi
dei capitelli; Absorbed le 6 facce di piano + i 4 fianchi verticali;
Kept il resto — la chiusura aritmetica verificata dal census-by-name).

#### D5.2 La sfera d'angolo — la derivazione completa

La palla di raggio r inscritta nell'angolo (1,1,1): tangente ai tre
piani x = 1, y = 1, z = 1 ⟹ centro C = (1−r, 1−r, 1−r), raggio r.
Tangente ai tre FIANCHI adiacenti: il fianco verticale ha l'asse
{(1−r, 1−r, z)} — C STA sull'asse, la tangenza sfera×cilindro è
COASSIALE, il cerchio di contatto nel piano z = 1−r, raggio r,
centrato C (il cerchio GRANDE della sfera: il parallelo a quota zero
del frame locale). Lo stesso, per permutazione, per i due fianchi del
ring (assi {(x, 1−r, 1−r)} e {(1−r, y, 1−r)} — C su entrambi): i tre
cerchi di contatto sono i tre grandi cerchi della sfera nei piani
x = 1−r, y = 1−r, z = 1−r. **Il patch è l'ottante sferico verso
l'angolo**: delimitato dai tre archi di quartiere (per esempio l'arco
da (1, 1−r, 1−r) a (1−r, 1, 1−r) sul cerchio z = 1−r — il quarto tra
i due punti di tangenza sui piani), i tre vertici I PUNTI DI TANGENZA
SUI PIANI ((1, 1−r, 1−r), (1−r, 1, 1−r), (1−r, 1−r, 1) — ognuno il
piede della perpendicolare da C al piano, il degenere landed
plane×sphere), 3 spigoli, 1 loop, la p-curva di ogni arco Circle2 sul
dominio della sfera (il parallelo del frame con polo sull'angolo — il
frame della faccia della sfera NASCE con l'atto, il polo la
bisettrice dell'angolo, la seam dichiarata al primo piano del naming)
e Line2 u = const sul dominio del cilindro adiacente. La superficie è
`SphereSurface(R = r, C)` — la famiglia esistente, la finestra
l'ottanto (θ ∈ [0, π/2], φ ∈ [0, π/2] nel frame locale): NESSUNA
superficie nuova, una finestra nuova.

**La chiralità, pin**: il frame della sfera d'angolo destro PER
COSTRUZIONE (il polo la bisettrice uscente, l'azimut di riferimento
la normale del primo piano del naming, il secondo asse il prodotto —
il costruttore deriva, il knob non esiste); l'ottanto camminato
antiorario visto dal polo (il material-on-the-left della legge della
casa), i pin nei golden (il digest della sfera, i versi degli archi).

#### D5.3 Le facce di piano, il confine finale (la derivazione dei rettangoli)

Ogni faccia di piano del box finisce come il RETTANGOLO delle rette
di tangenza: la top, per esempio, è delimitata da x = ±(1−r) (le
tangenti dei due fillet top∩right, top∩left) e y = ±(1−r) (top∩back,
top∩front), gli angoli del rettangolo I PUNTI DI TANGENZA DELLE
SFERE (per esempio (1−r, 1−r, 1) — il vertice condiviso con la sfera
e con i due archi). DERIVAZIONE: gli archi dei capitelli dei
verticali (i quarti di cerchio sui piani, centro (1−r, 1−r) raggio r)
giacciono INTERI nelle strisce che i fillet del ring rimuovono
(x ≥ 1−r o y ≥ 1−r per ogni punto interno dell'arco — verificado:
l'arco da (1, 1−r) a (1−r, 1) ha tutti i punti interni con x > 1−r E
y > 1−r), la regione della top dopo i ring è l'intersezione delle
semibande x ≤ 1−r, y ≤ 1−r (e simmetriche) — il rettangolo; il punto
(1−r, 1−r) è il centro dell'arco morto E il punto di tangenza della
sfera: la coincidenza è la chiusura della costruzione (la sfera tocca
il piano esattamente dove l'arco del capitello aveva il centro).

#### D5.4 La gate a ogni passo e il survival ledger

La catena è gated con K1 m5 a OGNI atto (i cinque predicati, la
tavola derivata, i riferimenti tenuti con le loro classi D3): il
passaggio verticale gate a ogni fillet (i conteggi della formula, la
tavola del fillet, la digest bit-identica al re-run); l'atto del
ring gate una volta (l'atto è atomico, la sua tavola conta TUTTE le
nascite e le morti — Born 8+8+…, Vanished gli spigoli e gli archi
morti, la chiusura aritmetica entrambi i lati). Il SURVIVAL LEDGER
sugli spigoli non ancora consumati (la domanda del brief): gli
spigoli del ring SOPRAVVISSUTI al passaggio verticale (i loro nomi
invariati — `box#1:top ∩ box#1:right` non è un nome della casa: gli
spigoli del box sono `box#1:vertical2`, gli orizzontali i ring del
ruolo della factory — i nomi esatti dal ruolo della tabella
kBoxEdgeRoles, il morso li pinna) tengono il loro nome attraverso il
passaggio (la classe 1 SILENT — il reference risolve identico), e
muoiono VANISHED nell'atto del ring col killer nominato (il fillet
che li ha mangiati); i TIMBRI delle facce assorbite sopravvivono
(box#1:top attraversa tutto, assorbita due volte, il nome del box
sempre); gli archi dei capitelli nascono e muoiono DENTRO la stessa
catena (la vita breve contata, la morte con la causa — "replaced by
the corner sphere's tangent arc"). Il ledger dell'edit m6 (il raggio
silenzioso) si estende: un edit di r del cubo intero ri-corre la
catena, gli stati a monte immutati, l'atto del ring cambiato, i
conteggi FISSI (24/48/26 per ogni r in banda), il volume la forma
chiusa al nuovo r (D7.2).

### D6. I muri dello stadio, con i criteri di STOP

1. **LA QUARTICA** (il muro centrale): se l'implementazione si trova
   a consultare cilindro×cilindro perpendicolari — o a introdurre una
   qualsiasi riga SSI nuova — STOP: la decomposizione atomica di D5.1
   è violata, il difetto è dell'implementazione, il rapporto nomina
   il passo.
2. **LA TOPOLOGIA DELL'ATTO** (il rischio reale): l'atto del ring
   nasce 16 facce (8 fianchi + 8 sfere) e ne assorbe/ri-finestra
   altre 10 con MORTI e NASCITE intrecciate (gli archi dei capitelli
   morti, i fianchi verticali ri-finestrati); se la tavola dell'atto
   non chiude l'aritmetica da entrambi i lati al primo tentativo —
   STOP, il rapporto con la riga che non torna, la correzione è un
   morso suo (mai cucita nel test).
3. **L'EDIT IN-PLACE DEL FIANCO VERTICALE** (il ri-finestramento): il
   fianco verticale cambia il bordo del capitello (l'arco morto
   sostituito dall'arco coassiale) SENZA morire — una faccia che
   cambia finestra tra due atti è l'edit del m6 generalizzato alle
   superfici di rivoluzione; se il resolver o la tavola non
   distinguono l'edit dalla morte — STOP (la classe D3 di ADR-0036
   deve dire Absorbed-con-rifrangimento, non Born-sottofalso).
4. **LA TESSELLAZIONE** (il debito del display): i nuovi membri del
   viewport (i fianchi X/Y, gli ottanti di sfera) chiedono la
   triangolazione watertight — il debito è NOMINATO nel registro G
   (il tessellator watertight); se il click-test mostra crepe agli
   archi — il difetto è del display, non del kernel: il rapporto lo
   separa.
5. **LA PORTA DEL FORMATO**: la ricetta multi-spigolo (la mappa
   spigolo→raggio) chiede una porta v0.8.0 (la legge di additività:
   ogni byte di v0.7 viaggia intero, i campi nominali, la disciplina
   della companion declaration di ADR-0007). Se l'arbitrazione del
   numero colpisce un'altra linea lo stesso giorno (il precedente
   v0.5/v0.6 di ADR-0039) — il morso REBASA, mai due bump lo stesso
   giorno.
6. **IL VERTEX BLEND** (la domanda che tornerà): raggi diversi allo
   stesso angolo — la risposta è NO con il nome
   (`fillet_corner_radii_mismatch`) e il rimedio; se un consumatore
   reale la chiede a gran voce, la sede è lo stadio 2/G3 col criterio
   d'ingresso, mai un'approssimazione di nascosto nel mondo esatto.

### D7. I golden proposti (pre-pinnati, la derivazione visibile)

#### D7.1 I conteggi

Lo stato finale del cubo intero a raggio unico: **V = 24, E = 48,
F = 26, H = 0, χ = 2** (D2.3 — la derivazione diretta, il censimento
dei co-edge 96/2, la chiusura di Euler). Gli stati intermedi del
passaggio verticale: 10/15/7, 12/18/8, 14/21/9, 16/24/10 (la formula
8+2k/12+3k/6+k, i primi due i golden esistenti IMMOVI). L'atto del
ring: la tavola contata (Born: 8 fianchi + 8 sfere + 24 rette di
tangenza + 24 archi + 24 vertici... la conta completa il morso la
deriva dalla tavola stessa, la chiusura aritmetica il test).

#### D7.2 Il volume — la forma chiusa di Steiner (l'apertura morfologica)

IL FILLET COSTANTE È L'APERTURA MORFOLOGICA: il corpo raccordato è
l'apertura del box per la palla B_r (erosione poi dilatazione — il
teorema classico del raccordo costante, la strada di CGM/CATIA
dietro il Rolling Ball). L'erosione del box [−1,1]³ è il nucleo
[−1+r, 1−r]³ (lato a = 2−2r); la dilatazione del nucleo è la SOMMA
DI MINKOWSKI nucleo ⊕ B_r — la scomposizione di Steiner in quattro
termini esatti:

    V(r) = a³ + 6·a²·r + 12·(πr²/4)·a + 8·(πr³/6)
         = a³ + 6a²r + 3π·a·r² + (4/3)πr³,     a = 2−2r

(i sei strati delle facce a²·r; i dodici quarti di cilindro
(πr²/4)·a; gli otto ottanti di sfera (1/8)·(4/3)πr³). LE VERIFICHE
AL LIMITI: r → 0 dà 8 (il box) ✓; **r = 1 dà (4/3)π — la sfera
inscritta del box, l'estremo dove i fianchi e le strisce muoiono
tutti e resta l'ottanto×8 = la sfera intera** ✓ (il collasso
tangenziale rifiutato in ammissione, ma la FORMA CHIUSA vede il
limite — la coerenza dei due fatti è il test dell'oracello). IL
GOLDEN NUMERICO: a r = 0.5, a = 1:

    V(0.5) = 1 + 3 + 0.75π + π/6 = 4 + 11π/12 ≈ 6.879793265790644

(la forma chiusa ESATTA: 4 + 11π/12 — la tolleranza 1e-12, le due
strade — la scomposizione di Steiner e il teorema della divergenza
del kernel — d'accordo al pin). Il contro-tests: il volume del box
con UN fillet verticale (7.5 + π/8, il golden esistente) è il caso
k = 1 della stessa fisica — la coerenza tra le due forme chiuse (il
cuneo di D1.1 e Steiner) è verificata numericamente nel morso
dell'oracolo (la scomposizione di Steiner con k strisce... il
controllo incrociato: V(1 fillet) = 8 − 2r²(1−π/4)·ℓ... le strade
diverse convergono — l'oracolo indipendente a zero codice condiviso
condivide solo il π).

#### D7.3 Le parziali e la tolleranza

Le parziali (i 4 verticali con raggi r₁..r₄: i volumi della formula
additiva dei cunei — 8 − Σᵢ ℓᵢ·rᵢ²(1−π/4) con ℓᵢ = 2, le condizioni
di striscia verificate in ammissione; un ring solo: la stessa formula
un asse sotto) — i golden secondari, la derivazione dal walk. La
TOLLERANZA: 1e-12 per le forme chiuse (le due strade esatte in
doppia precisione, il pin della casa da m3/m7), mai allargata per
passare.

#### D7.4 Gli oracoli indipendenti (il protocollo, la quinta riga)

L'ORACELLO DI MASSA a zero codice condiviso col kernel (la legge D1.5
di ADR-0041): la scomposizione di Steiner scritta NEL TEST con la sua
provenienza (il teorema dell'apertura morfologica, la formula dei
quattro termini derivata nel commento), la quadratura di controllo
(la strada numerica: il cubo campionato, l'apertura valutata per
punti — 2e6 campioni, la tolleranza relativa 2e-6 dichiarata come in
m7), il confronto col kernel (il teorema della divergenza sulle
facce del corpo) — TRE strade, zero codice condiviso. Gli oracoli
geometrici: i 24 vertici alle posizioni chiuse (le tre permutazioni
per angolo), le 24 rette di tangenza (posizioni e direzioni), i 24
archi (i centri, i piani, i quarti), le 8 sfere (i centri
(±(1−r), ±(1−r), ±(1−r)), i raggi r) — TUTTI derivati nel test, mai
copiati dal kernel.

### D8. Il protocollo di famiglia — le sei righe sul censimento, il delta contro il baseline

La famiglia estesa NON è una riga nuova del censimento: è
L'APPROFONDIMENTO delle righe esistenti (la legge di ADR-0041: le
famiglie contate, non le feature — e le superfici sono le stesse):

1. **Cylinder (la riga esistente, 6 PASS / 1 dash)**: il writer guadagna
   i siti X/Y (i fianchi del ring nascono dai fillet — il terzo genere
   di sito dopo la factory e l'estrusione); il frame e la chiralità le
   costruzioni D2.1 (i pin nei test della famiglia, il pattern di
   test_analytic_surfaces un asse sopra); la massa la forma chiusa del
   quarto di cilindro (il contributo del fianco al teorema della
   divergenza — l'oracolo D7.4); le bande il θ-quarto con lo Jacobiano
   r; i degeneri: il dash dei degeneri della riga RESTA (la seconda
   coorte dello zoo non è questo stadio — la sede dichiarata); la
   serializzazione la ricetta D3.5. **Il delta: due celle (writer,
   band) guadagnano prove, nessuna cella si muove indietro.**
2. **Sphere (la riga esistente, 6 PASS / 1 dash)**: il writer guadagna
   il sito sfera-d'angolo (il quarto genere dopo factory/cielo... il
   terzo: la factory make_sphere e ora l'ottanto del fillet); il
   frame il polo-bisettrice D5.2; la massa l'ottanto (1/8)·(4/3)πr³ —
   l'oracolo classico di Archimede un ottante sopra; il dash dei
   degeneri resta. **Il delta: una cella (writer) guadagna prove.**
3. **Le altre righe: IMMOSSE.** Il piano non cambia (le finestre
   rettangolari sono la stessa disciplina), il toro non entra, il
   cono resta il cono (il suo dash, la sede nominata), l'ellissoide
   la riga completa che era.
4. **Lo zoo**: le coppie NUOVE che la costruzione tocca sono solo le
   tre di D3 (tutte landed o by-construction) — NESSUNA nuova coppia
   per il censimento dei degeneri; la verifica della tesi D3 (la
   quartica mai consultata) è un test dello stadio (lo spy sul
   consulto — o l'assenza della riga nel diff, la prova strutturale).
5. **La chain gate**: la catena del cubo (i 5 atti: 4 verticali + 1
   atto ring... o 2 atti: il passaggio verticale batch + l'atto del
   ring) gated a ogni passo — il membro della suite m5 si estende ai
   nuovi stati (16/24/10, 24/48/26), la digest bit-identica al re-run.
6. **Il benchmark**: la riga del profilo (il gesto → il corpo
   ricostruito) sul cubo intero — il numero accanto ai cinque membri
   esistenti, la ratio la sostanza.

**Il delta stimato contro il baseline (il censimento a 42 celle)**:
ZERO celle nuove, DUE celle guadagnano prove (il writer del cilindro
— i siti X/Y; il writer della sfera — gli ottanti), il dash dei
degeneri del cilindro e della sfera resta coi suoi sederi (la seconda
coorte dello zoo, non questo stadio). Il censimento NON si muove a
34 PASS: le celle erano già PASS — le loro PROVE si approfondiscono
(la legge di ADR-0041: il censimento inventa nessun test, rifiuta che
i test esistenti non si vedano).

### D9. La stima e i morsi

Le unità di lavoro, contate alla scala della casa (i morsi del
fillet ADR-0039 furono 6; l'ellissoide 3; la chiusura K1 1):

1. **Le superfici X/Y e i loro pin** (le classi, i frame, la
   chiralità, i test della famiglia un asse sopra) — un morso.
2. **Il fianco del ring** (la costruzione chiusa, il walk, la
   formula, i golden 10/15/7 un asse sopra) — un morso.
3. **La sfera d'angolo e l'atto atomico** (i cerchi coassiali, l'ottanto,
   la tavola dell'atto, la morte degli archi, l'edit dei verticali) —
   DUE morsi (il rischio D6.2/3 vive qui).
4. **La ricetta multi-spigolo e la porta v0.8.0** (la mappa
   spigolo→raggio, i campi nominali, il migrator, i golden del
   formato) — un morso.
5. **La suite** (i golden D7, la gate walk, il ledger, l'oracolo a
   zero codice condiviso, il benchmark, le controprove — il metodo
   della chiusura K1 applicato al caso nuovo) — un morso.
6. **Il click-test e il registro** — mezzo morso, la riga finale.

TOTALE STIMATO: 6–7 morsi. I CRITERI DI STOP di D6 vincolano ognuno;
il primo morso è le superfici X/Y (nessun rischio, la fondazione);
il terzo è il cuore e il rischio.

**IL CLICK-TEST (il settimo, l'uscita)**: il cubo con TUTTI i 12
spigoli raccordati e gli 8 angoli a sfera, VISIBILE nel viewport (i
fianchi e gli ottanti renderizzati watertight — il debito del
display affrontato) e MISURABILE (il pannello dice il volume
6.879793265790644 a r = 0.5, il report dice 24/48/26) — la frase del
maintainer che chiude lo stadio: "riesco a raccordare tutti gli
spigoli del cubo, verticali e del ring, e gli angoli sono rotondi."

## Consequences

- Il muro vertical-only cade per il cubo; il picker passa da 4-in a
  12-in (con la legge degli angoli dichiarata prima di ogni Apply —
  la stessa onestà di ADR-0039, la riga del picker aggiornata).
- La tabella SSI non cambia di una riga (D3/D4 — il claim
  verificabile dal diff).
- Il formato guadagna la v0.8.0 (la mappa spigolo→raggio, i campi
  nominali, l'additività di v0.7 intera) — l'arbitrazione del numero
  al suo tempo (il precedente v0.5/v0.6).
- Lo stadio 2 (il dihedro generale) ha il suo criterio di ingresso
  (D1-OUT): il consumatore reale non ortogonale, la derivazione di
  ADR-0039 D1.1 un parametro più larga — la finestra del capitello
  π−α e la tangente r·cot(α/2) già scritte.
- Il vertex blend variabile ha la sua porta dichiarata (G3, il mondo
  approssimato, la provenance gate) — mai dentro il mondo esatto.

## References

- ADR-0039 (il fillet: D1.1 la classe box-edge e la formula della
  catena; D4 la coda — crease merge, vertex blend, la legge delle
  strisce), ADR-0041 (il protocollo di famiglia: le sei righe, il
  censimento, gli oracoli a zero codice condiviso), ADR-0036 (il
  naming, le classi D3, la tavola), ADR-0033 D2 (il criterio
  both-domain, la quartica di Steinmetz), ADR-0023 (la seam
  slit-rectangle), ADR-0007/0027 (le porte del formato, la
  companion declaration).
- La catena m6ter (tests/unit/brep/test_g2_u3_m6ter_the_chain.cpp:
  i conteggi 8/12/6 → 10/15/7 → 12/18/8, i nomi annidati), le
  tabelle SSI (core/brep/include/CAD_0/brep/ssi.hpp: le righe
  landed, i muri, la disposizione corretta dei raggi disuguali, la
  provenance gate; tests/unit/brep/test_g2_unit1_ssi_table.cpp: i
  degeneri tipizzati), la suite della chiusura K1
  (tests/unit/brep/test_k1_closure_consumption.cpp: il metodo delle
  sette colonne che il morso della suite applica al caso nuovo).
