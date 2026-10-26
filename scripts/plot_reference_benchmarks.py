#!/usr/bin/env python3
"""Plot retained offline reference diagnostics; never labels them as telemetry."""
import argparse
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('results', type=Path)
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args()
    cases = json.loads(args.results.read_text())['cases']
    selected = [r for r in cases if r['case'] in (
        'straight-speed-2', 'reversal', 'five-stops', 'figure-eight', 'dense-10000', 'vin', 'circle-1-lap')]
    names = [r['case'] for r in selected]
    duration = np.array([r['independent_reference_audit']['duration_s'] for r in selected])
    stationary = np.array([r['independent_reference_audit']['sampled_stationary_time_s'] for r in selected])
    fig, axes = plt.subplots(1, 2, figsize=(12, 5), layout='constrained')
    axes[0].barh(names, duration-stationary, label='Moving', color='#187a9b')
    axes[0].barh(names, stationary, left=duration-stationary, label='Stationary / turning', color='#d59126')
    axes[0].set_xlabel('Planned route time (s), sampled classification')
    axes[0].legend(loc='lower right')
    axes[1].barh(names, [r['elapsed_s'] for r in selected], color='#187a9b')
    axes[1].set_xlabel('Compilation + audits (wall seconds, this host)')
    fig.suptitle('Offline reference benchmarks — no simulator or measured flight')
    args.output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(args.output, dpi=160)


if __name__ == '__main__':
    main()
