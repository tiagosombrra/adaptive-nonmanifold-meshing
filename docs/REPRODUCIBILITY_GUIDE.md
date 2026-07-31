# Reproducibility guide

## Scope

The representative scientific result is Figure 7, Section 4.3, “Eistute model”, from the associated *Computers & Graphics* article.

The figure contains five saved mesh states numbered 0–4. In the configuration file, `ADAPTIVE_MAX_STEPS=4` denotes the highest saved stage index.

The regression target is the saved stage-4 candidate mesh. Rejected adaptive candidates are intentionally retained for analysis, so the stage-4 object is verified independently of the hybrid transition-acceptance decision.

## Exact target

The representative verifier requires:

- configured final stage: 4;
- no mesh stage greater than 4;
- stage-4 mesh present;
- triangular faces present;
- stage-4 triangle count: 7,184.

The archived scientific metadata also report:

| Metric | Value |
|---|---:|
| Normalized error | 0.036712 |
| Total error reduction | 96.3288% |
| Triangles with quality `q >= 0.60` | 58.6303% |
| Mean shared-curve compatibility error | 0.0 |

## Linux workflow

### Requirements

- Ubuntu 22.04 or compatible Linux;
- CMake 3.15 or later;
- C++17 compiler;
- Python 3.

Install the Ubuntu dependencies:

```bash
sudo apt-get update
sudo apt-get install -y build-essential cmake python3
```

### One-command reproduction

```bash
chmod +x reproduce_eistute_linux.sh scripts/*.sh
bash reproduce_eistute_linux.sh
```

Expected verification output includes:

```text
Eistute final stage verified: 4
Triangles: 7184
Published Eistute target triangle count verified.
```

Scientific results are written under:

```text
results/eistute/
```

The representative OBJ filename ends with:

```text
passo_4_malha_4.obj
```

### Build controls

The Linux build supports:

```text
AP_MESH_BUILD_DIR
AP_MESH_BUILD_JOBS
AP_MESH_CLEAN_BUILD
```

For example:

```bash
AP_MESH_BUILD_JOBS=1 bash reproduce_eistute_linux.sh
```

## Windows workflow

The inherited GRSI workflow remains available for Windows 10/11 with MSYS2 UCRT64, GCC/MinGW-w64, CMake and Python 3.

From PowerShell:

```powershell
.\reproduce_eistute_windows.bat
```

From MSYS2 UCRT64 or Git Bash:

```bash
cmd.exe //c reproduce_eistute_windows.bat
```

Complete clean-machine installation instructions are available in [`../INSTALL_WINDOWS.txt`](../INSTALL_WINDOWS.txt).

## container environment-style workflow

Run:

```bash
bash reproduce_eistute_linux.sh
```

The workflow compiles, executes, verifies and packages the result. In a container environment environment, the package is written to `/results`. For local execution:

```bash
AP_MESH_RESULTS_DIR="$PWD/codeocean-results" bash reproduce_eistute_linux.sh
```

The portable result package contains:

```text
eistute_stage4.obj
run.log
verification.txt
result_metadata.json
```

The metadata file must contain:

```json
{
  "stage": 4,
  "triangles": 7184,
  "expected_triangles": 7184,
  "verified": true
}
```

## Container workflow

Build:

```bash
docker build -f container/Dockerfile -t adaptive-nonmanifold-meshing .
```

Run:

```bash
mkdir -p docker-results

docker run --rm \
  -v "$PWD/docker-results:/results" \
  adaptive-nonmanifold-meshing
```

## Archived evidence

The repository includes:

- `reference/eistute_stage4.obj`;
- `reference/eistute_metrics.csv`;
- `reference/eistute_stage4.json`.

Verify their internal consistency without executing the mesher:

```bash
python3 scripts/verify_reference.py
```

## Automated continuous integration

The workflow `.github/workflows/linux-reproduction.yml` runs the Linux stages independently:

1. build;
2. execute and verify;
3. package container environment results;
4. upload the generated artifact.

The required future branch-protection check will be named:

```text
linux-build-and-test
```

It should be made mandatory only after the OSP preparation pull request is ready to leave draft status.

## Additional models

Book and Decor Shelf scripts are included for Linux and Windows. Their current validation is structural. The exact archived numerical target for regression testing remains the Eistute stage-4 mesh.

## Numerical diagnostics

The execution log may report `FindUV` calls that reach the configured iteration limit. This counter is retained as a numerical diagnostic and is not part of the triangle-count acceptance criterion.

Legacy internal timing fields are also diagnostic. Use an external wall-clock measurement when execution time must be reported.
