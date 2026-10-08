# Istruzioni per la compilazione e creazione di eseguibili

CAD_0 supporta due tipi di eseguibili:

1. **Eseguibile C++ headless** — per batch processing, CI/CD, scripting
2. **Applicazione desktop Python (PySide6)** — per uso interattivo con UI

---

## 1. Prerequisiti comuni

### Linux (Debian/Ubuntu)

```bash
sudo apt install build-essential cmake ninja-build python3 python3-dev python3-pip
```

### Windows

- **Visual Studio 2022** (con workload "Desktop development with C++")
- **Python 3.10+** da https://python.org (spunta "Add to PATH")
- **CMake** da https://cmake.org/download/ o `pip install cmake`
- **Ninja** (opzionale, raccomandato): `pip install ninja`

### Python (entrambe le piattaforme)

```bash
pip install cmake ninja nanobind scikit-build-core matplotlib numpy
```

Per l'app desktop:
```bash
pip install PySide6 pyinstaller
```

---

## 2. Build C++ headless (Linux)

L'eseguibile C++ è già auto-contenuto: le librerie CAD_0 sono statiche
(`.a`) e le dipendenze (fmt, spdlog, xsimd) sono scaricate via CPM e
linkate staticamente. L'eseguibile finale dipende solo da librerie di
sistema (libstdc++, libc, libm).

### Build standard

```bash
cd CAD_0
cmake -B build -G Ninja \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_POLICY_VERSION_MINIMUM=3.5 \
    -DCAD_0_BUILD_PYTHON=OFF \
    -DCAD_0_BUILD_TESTING=ON \
    -DCAD_0_BUILD_EXAMPLES=ON \
    -DCAD_0_USE_TBB=OFF

cmake --build build
```

### Output

```
build/bin/example_sphere_to_stl   # Eseguibile standalone (106 KB)
build/bin/CAD_0_tests              # Test runner
```

### Verifica dipendenze

```bash
ldd build/bin/example_sphere_to_stl
```

Deve mostrare solo librerie di sistema (libstdc++, libc, libm).
Nessuna dipendenza dinamica da librerie CAD_0 o di terze parti.

### Test

```bash
./build/bin/CAD_0_tests           # 165 test, devono passare tutti
./build/bin/example_sphere_to_stl # Genera STL/OBJ/3MF
```

---

## 3. Build C++ headless (Windows)

### Con Visual Studio (MSVC)

```powershell
cd CAD_0
cmake -B build -G "Visual Studio 17 2022" -A x64 ^
    -DCMAKE_BUILD_TYPE=Release ^
    -DCMAKE_POLICY_VERSION_MINIMUM=3.5 ^
    -DCAD_0_BUILD_PYTHON=OFF ^
    -DCAD_0_BUILD_TESTING=ON ^
    -DCAD_0_BUILD_EXAMPLES=ON ^
    -DCAD_0_USE_TBB=OFF

cmake --build build --config Release
```

### Con Ninja (se installato)

```powershell
cd CAD_0
cmake -B build -G Ninja ^
    -DCMAKE_BUILD_TYPE=Release ^
    -DCMAKE_POLICY_VERSION_MINIMUM=3.5 ^
    -DCAD_0_BUILD_PYTHON=OFF ^
    -DCAD_0_BUILD_TESTING=ON ^
    -DCAD_0_BUILD_EXAMPLES=ON ^
    -DCAD_0_USE_TBB=OFF ^
    -DCMAKE_CXX_COMPILER=cl

cmake --build build
```

### Output

```
build\bin\Release\example_sphere_to_stl.exe   # Eseguibile Windows
build\bin\Release\CAD_0_tests.exe              # Test runner
```

### Note Windows

- I file `.exe` dipendono da `MSVCP140.dll` e `VCRUNTIME140.dll`
  (Visual C++ Redistributable). Per evitare questo:
  - Opzione A: installare il [VC++ Redistributable](https://aka.ms/vs/17/release/vc_redist.x64.exe)
  - Opzione B: static link del runtime aggiungendo `/MT` ai flag del compilatore:
    ```cmake
    set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>")
    ```
    Aggiungere questa riga al `CMakeLists.txt` prima di `project()`.

---

## 4. Build con Python bindings (Linux)

```bash
cd CAD_0
cmake -B build -G Ninja \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_POLICY_VERSION_MINIMUM=3.5 \
    -DCAD_0_BUILD_PYTHON=ON \
    -DCAD_0_BUILD_TESTING=ON \
    -DCAD_0_USE_TBB=OFF \
    -DPython3_EXECUTABLE=$(which python3)

cmake --build build
```

### Output

```
build/bin/CAD_0_tests                           # Test C++
build/bin/example_sphere_to_stl                # Eseguibile C++ headless
build/python/CAD_0/_CAD_0.cpython-312-*.so     # Estensione Python compilata
build/python/CAD_0/                             # Pacchetto Python completo
```

### Test Python

```bash
cd build/python
PYTHONPATH=. python3 -c "
import CAD_0
sph = CAD_0.sdf.sphere(radius=2.0)
mesh = CAD_0.sdf.marching_cubes(sph, resolution=32)
print(f'{mesh.vertex_count()} verts, {mesh.triangle_count()} tris')
CAD_0.io.export('/tmp/test.stl', mesh)
print('OK')
"
```

---

## 5. Build con Python bindings (Windows)

```powershell
cd CAD_0
cmake -B build -G Ninja ^
    -DCMAKE_BUILD_TYPE=Release ^
    -DCMAKE_POLICY_VERSION_MINIMUM=3.5 ^
    -DCAD_0_BUILD_PYTHON=ON ^
    -DCAD_0_BUILD_TESTING=ON ^
    -DCAD_0_USE_TBB=OFF ^
    -DPython3_EXECUTABLE=python

cmake --build build
```

### Output

```
build\python\CAD_0\_CAD_0.cp312-win_amd64.pyd   # Estensione Python
build\python\CAD_0\                              # Pacchetto Python completo
```

---

## 6. Creazione applicazione desktop standalone (PyInstaller)

Dopo aver compilato i binding Python (sezione 4 o 5), usa PyInstaller
per creare un eseguibile standalone che include Python, PySide6 e
l'estensione compilata.

### Step 1: Installa PySide6

```bash
pip install PySide6 pyinstaller
```

### Step 2: Crea lo spec file PyInstaller

Crea un file `CAD_0.spec` nella root del progetto:

```python
# CAD_0.spec — PyInstaller spec for CAD_0 desktop app
import os
import sys
from PyInstaller.utils.hooks import collect_data_files

block_cipher = None

# Path alla build Python (adatta al tuo sistema)
BUILD_PY = os.path.join('build', 'python')

a = Analysis(
    ['app/src/CAD_0/ui/__main__.py'],
    pathex=[BUILD_PY],
    binaries=[],
    datas=[
        # Includi il pacchetto Python compilato
        (os.path.join(BUILD_PY, 'CAD_0'), 'CAD_0'),
        # Includi gli shader GLSL
        ('core/viewport/include/CAD_0/viewport/glsl', 'CAD_0/viewport/glsl'),
    ],
    hiddenimports=[
        'CAD_0._CAD_0',
        'CAD_0.math',
        'CAD_0.geometry',
        'CAD_0.sdf',
        'CAD_0.io',
        'CAD_0.scene',
        'CAD_0.camera',
        'CAD_0.viewer',
        'PySide6.QtOpenGL',
        'matplotlib',
        'matplotlib.backends.backend_qtagg',
    ],
    hookspath=[],
    runtime_hooks=[],
    excludes=['tkinter', 'PyQt5', 'PyQt6'],
    cipher=block_cipher,
)

pyz = PYZ(a.pure, a.zipped_data, cipher=block_cipher)

exe = EXE(
    pyz,
    a.scripts,
    [],
    exclude_binaries=True,
    name='CAD_0',
    debug=False,
    strip=False,
    upx=True,
    console=False,        # App GUI, nessuna console
    icon='app/resources/icons/cad_0.ico',  # Se disponibile
)

coll = COLLECT(
    exe,
    a.binaries,
    a.zipfiles,
    a.datas,
    strip=False,
    upx=True,
    name='CAD_0',
)
```

### Step 3: Compila l'eseguibile

```bash
# Assicurati che la build Python sia aggiornata
cmake --build build

# Crea l'eseguibile con PyInstaller
pyinstaller CAD_0.spec --clean --noconfirm
```

### Output

```
dist/CAD_0/CAD_0              # Eseguibile Linux (o CAD_0.exe su Windows)
dist/CAD_0/                   # Cartella con tutte le dipendenze
```

### Step 4 (opzionale): Crea un singolo file auto-estrante

```bash
pyinstaller CAD_0.spec --clean --noconfirm --onefile
```

Produce un singolo file `dist/CAD_0` (o `.exe`) che contiene tutto.
Più lento all'avvio (~3 secondi per decompressione) ma più semplice da distribuire.

---

## 7. Script di build automatico

Per comodità, usa lo script incluso:

### Linux

```bash
./scripts/build.sh --python --test --clean
```

### Build completa (C++ + Python + Desktop app)

```bash
#!/bin/bash
set -e

cd CAD_0

# 1. Build C++ + Python
cmake -B build -G Ninja \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_POLICY_VERSION_MINIMUM=3.5 \
    -DCAD_0_BUILD_PYTHON=ON \
    -DCAD_0_BUILD_TESTING=ON \
    -DCAD_0_USE_TBB=OFF \
    -DPython3_EXECUTABLE=$(which python3)

cmake --build build

# 2. Test
./build/bin/CAD_0_tests

# 3. Test Python
cd build/python
PYTHONPATH=. python3 -c "import CAD_0; print('Python OK')"
cd ../..

# 4. App desktop (PyInstaller)
pyinstaller CAD_0.spec --clean --noconfirm

echo "Build complete!"
echo "  Eseguibile C++:    build/bin/example_sphere_to_stl"
echo "  Estensione Python: build/python/CAD_0/_CAD_0*.so"
echo "  App desktop:       dist/CAD_0/CAD_0"
```

---

## 8. Distribuzione

### Linux

| Componente | File | Dipendenze runtime |
|---|---|---|
| Eseguibile C++ | `example_sphere_to_stl` | libstdc++, libc (preinstallate) |
| App desktop | `dist/CAD_0/CAD_0` | nessuna (tutto incluso) |

Per l'app desktop, distribuisci l'intera cartella `dist/CAD_0/`.
L'utente deve solo avere un display server (X11 o Wayland).

### Windows

| Componente | File | Dipendenze runtime |
|---|---|---|
| Eseguibile C++ | `example_sphere_to_stl.exe` | VC++ Redistributable 2015-2022 |
| App desktop | `dist/CAD_0/CAD_0.exe` | nessuna (tutto incluso) |

Per l'app desktop, distribuisci l'intera cartella `dist/CAD_0/`.
Su Windows puoi anche creare un installer con NSIS o Inno Setup.

---

## 9. Troubleshooting

### "Could NOT find Python3 (missing: Development.Module)"

```bash
# Linux
sudo apt install python3-dev

# Venv: passa esplicitamente il path
-DPython3_EXECUTABLE=/path/to/venv/bin/python3
```

### TBB build fallisce con GCC 12+ (-Werror)

TBB è disabilitato di default (`CAD_0_USE_TBB=OFF`). Il progetto funziona
perfettamente senza TBB (fallback single-threaded). Abilita TBB solo se
hai oneTBB installato system-wide:

```bash
# Linux
sudo apt install libtbb-dev

# Build con TBB system-wide (non CPM)
cmake -B build -DCAD_0_USE_TBB=ON -DCPM_USE_LATEST_VERSION=OFF
```

### PyInstaller: "ModuleNotFoundError: No module named 'CAD_0._CAD_0'"

Assicurati che il path `BUILD_PY` nello spec file punti alla directory
corretta (`build/python`). L'estensione compilata (`.so` o `.pyd`)
deve essere nella cartella `CAD_0/` dentro il path di build.

### PyInstaller: app si apre ma la finestra è nera

Verifica che OpenGL sia disponibile sul sistema. Su Linux in headless/CI:
```bash
# Usa Mesa software rendering
LIBGL_ALWAYS_SOFTWARE=1 ./dist/CAD_0/CAD_0
```

### Windows: "MSVCP140.dll not found"

Installa il Visual C++ Redistributable:
https://aka.ms/vs/17/release/vc_redist.x64.exe

Oppure static-link il runtime (vedi sezione 3, "Note Windows").

---

## 10. CI/CD (GitHub Actions)

Il progetto include un workflow CI in `.github/workflows/ci-linux.yml`.
Per build su Windows, aggiungi un file `.github/workflows/ci-windows.yml`:

```yaml
name: ci-windows
on: [push, pull_request]
jobs:
  build:
    runs-on: windows-2022
    steps:
      - uses: actions/checkout@v4
      - uses: actions/setup-python@v5
        with:
          python-version: '3.12'
      - run: pip install cmake ninja nanobind scikit-build-core
      - run: cmake -B build -G Ninja -DCMAKE_BUILD_TYPE=Release -DCMAKE_POLICY_VERSION_MINIMUM=3.5 -DCAD_0_BUILD_PYTHON=OFF -DCAD_0_BUILD_TESTING=ON -DCMAKE_CXX_COMPILER=cl
        shell: cmd
      - run: cmake --build build
      - run: .\build\bin\CAD_0_tests.exe
        shell: cmd
```

---

## Riepilogo rapido

| Piattaforma | C++ headless | App desktop Python |
|---|---|---|
| **Linux** | `cmake -B build && cmake --build build` → `build/bin/example_sphere_to_stl` | + `pyinstaller CAD_0.spec` → `dist/CAD_0/CAD_0` |
| **Windows** | `cmake -B build -G "Visual Studio 17 2022" && cmake --build build --config Release` → `build\bin\Release\example_sphere_to_stl.exe` | + `pyinstaller CAD_0.spec` → `dist\CAD_0\CAD_0.exe` |

Il progetto è strutturato per essere cross-platform:
- C++20 standard (funziona su GCC, Clang, MSVC)
- CMake build system (funziona su Linux, Windows, macOS)
- Le librerie CAD_0 sono tutte statiche (nessuna DLL da distribuire)
- L'estensione Python è un singolo file `.so`/`.pyd`
