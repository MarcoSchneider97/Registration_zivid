# Point-Cloud Stitching (`stitching/`)

Dieses Verzeichnis enthält ein kleines CLI-Tool zum iterativen Stitching mehrerer `.ply`-Punktwolken mit der Zivid-Registrierungs-API.

## Struktur

- `stitch_point_clouds.py` – Hauptskript
- `data/` – Eingabedaten (`*.ply`)
- `output/` – Ergebnisse
  - `transforms/` – Pro Schritt eine 4x4 Transformationsmatrix als CSV

## Dateikonvention

- Eingaben: Alle Dateien in `data/` mit Endung `.ply` werden **lexikographisch sortiert** verarbeitet.
  - Erste Datei = initiales `target`
  - Jede weitere Datei = `source`, die auf aktuelles `target` registriert wird
- Transformationen: `output/transforms/transform_001.csv`, `transform_002.csv`, ...
- Finale Punktwolke: standardmäßig `output/stitched.ply` (konfigurierbar via `--output`)

## Ablauf (API-nah)

1. Lade alle `*.ply` als `UnorganizedPointCloud`.
2. Optionales Voxel-Downsampling für die Registrierung (Performance-Empfehlung aus der API-Doku).
3. Iterativ:
   - `target = first`
   - Für jede weitere Cloud:
     - `local_point_cloud_registration(target, source, parameters)`
     - Ergebnis-Transform auf die **volle** Source anwenden
     - `target.extend(source)`
4. Speichere pro Schritt die Transform als CSV und am Ende die gestitchte Punktwolke als `.ply`.

## Verwendung

Im Repo-Root ausführen (typisch mit gesetztem `PYTHONPATH=modules`):

```bash
PYTHONPATH=modules python stitching/stitch_point_clouds.py
```

### Parameter

```bash
PYTHONPATH=modules python stitching/stitch_point_clouds.py \
  --voxel-size 1.0 \
  --min-points-per-voxel 1 \
  --max-correspondence-distance 5.0 \
  --max-iteration-count 100 \
  --output stitched.ply
```

- `--voxel-size` *(optional)*: Aktiviert Downsampling für die Registrierung.
- `--min-points-per-voxel`: Mindestanzahl Punkte pro Voxel (nur relevant mit `--voxel-size`).
- `--max-correspondence-distance`: Überschreibt Registrierungsparameter.
- `--max-iteration-count`: Überschreibt Registrierungsparameter.
- `--output`: Ausgabedatei (`stitched.ply` default). Relative Pfade werden relativ zu `output/` aufgelöst.
- `--data-dir`, `--output-dir`: Optional zur Anpassung der Verzeichnisse.

## Hinweise

- Für stabile Registrierung sollten benachbarte Scans ausreichend Überlappung haben.
- Downsampling reduziert Laufzeit deutlich; finale Geometrie bleibt dicht, da Transform auf die ungedownsamplete Source angewandt wird.
