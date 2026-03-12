# Documentation (Read the Docs)

This directory contains the source for the **public documentation** hosted on [Read the Docs](https://readthedocs.org).

## Building locally

```bash
pip install -r docs/requirements.txt
mkdocs serve
```

Open [http://127.0.0.1:8000](http://127.0.0.1:8000).

## Read the Docs setup

1. Import the project at [readthedocs.org](https://readthedocs.org/dashboard/import/).
2. Connect your Git repository (GitHub, GitLab, etc.).
3. Read the Docs will use `.readthedocs.yaml` and `mkdocs.yml` automatically.
4. Update `site_url` in `mkdocs.yml` after creating the RTD project.

## Documentation scope

- **Free edition** — detailed implementation documentation covering all solver formulations, zero-injection handling, post-estimation evaluation, the REST/gRPC API, and the Angular frontend.
- **Pro edition** — capabilities overview describing *what* the Observability, Redundancy, and Bad Data Detection modules can do, without documenting *how* they are implemented.

## File structure

| File | Purpose |
|------|---------|
| `index.md` | Home page — overview of both editions |
| `quickstart.md` | Build, run, and usage instructions |
| `architecture.md` | Module structure, solver formulations, zero-injection, post-estimation |
| `api.md` | Complete REST API reference (free edition) |
| `sequence-diagram.md` | Mermaid sequence diagrams for startup and estimation flow |
| `case-study.md` | GB network case study overview |
| `pro.md` | Pro edition capabilities (Observability, Redundancy, BDD) |
| `references.md` | Academic references |

## How to cite

If you use Rusty Givens in academic work, please cite:

**BibTeX**

```bibtex
@software{rusty_givens,
  author = {Jendrian, Harald},
  title = {Rusty Givens: A modular power system state estimator in Rust},
  year = {2025},
  url = {https://github.com/haraldjendrian/rusty-givens}
}
```

**Plain text**

> Jendrian, H. (2025). Rusty Givens: A modular power system state estimator in Rust. https://github.com/haraldjendrian/rusty-givens
