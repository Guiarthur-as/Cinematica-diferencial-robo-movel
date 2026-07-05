# Relatório LaTeX

Relatório técnico do trabalho de Cinemática Diferencial e Simulação de
Robôs Móveis.

## Arquivos

- `relatorio.tex` — documento principal (deduções, metodologia, resultados);
- `tikz/` — figuras de geometria vetoriais (TikZ):
  - `diffdrive_geometry.tex` — geometria do robô diferencial (com roda boba);
  - `ackermann_geometry.tex` — geometria da direção de Ackermann;
  - `icr.tex` — Centro Instantâneo de Rotação.

## Pré-requisitos

- Uma distribuição TeX (testado com **MiKTeX**; TeX Live também funciona);
- Pacotes: `babel` (brazilian), `amsmath`, `graphicx`, `tikz`, `siunitx`,
  `booktabs`, `subcaption`, `float`, `hyperref`, `geometry`. No MiKTeX são
  instalados automaticamente sob demanda.

## Antes de compilar

As figuras de resultados vêm de `../outputs/` (referenciadas via
`\graphicspath{{../outputs/}}`). Gere-as primeiro, a partir da raiz do
projeto:

```bash
uv run python run_simulations.py
```

## Compilação

Na pasta `relatorio/` (o documento não usa bibliografia, então basta o
`pdflatex`):

```bash
pdflatex relatorio.tex
pdflatex relatorio.tex
```

Ou, se tiver `latexmk`:

```bash
latexmk -pdf relatorio.tex
```

O resultado é `relatorio.pdf`. Um PDF de referência já acompanha a pasta;
substitua-o pela sua versão final se desejar.
