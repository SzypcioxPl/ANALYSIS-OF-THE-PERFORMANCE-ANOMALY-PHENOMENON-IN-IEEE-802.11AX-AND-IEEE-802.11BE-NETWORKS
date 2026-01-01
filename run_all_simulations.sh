#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$SCRIPT_DIR"
LOG_DIR="$PROJECT_ROOT/scratch/logs"
BE_MAX_AMPDU=4194304      # domyślna wartość w bajtach (0 wyłącza A-MPDU)  4194304 65535 262144
SIMULATION_TIME=260     # domyślny czas symulacji (s)
CLIENT_INTERVAL=0.0001  # domyślny odstęp między pakietami (s)

mkdir -p "$LOG_DIR"

SCENARIOS=(
  scenario_coex_a_ax
  scenario_coex_a_ax_dualsta
  scenario_coex_a_ax_triplesta
  scenario_coex_a_ax_quadsta
  scenario_coex_a_ax_decsta
  scenario_coex_a_be
  scenario_coex_a_be_dualsta
  scenario_coex_a_be_quadsta
  scenario_coex_a_be_decsta
  scenario_coex_ac_ax
  scenario_coex_ac_ax_dualsta
  scenario_coex_ac_ax_triplesta
  scenario_coex_ac_ax_quadsta
  scenario_coex_ac_ax_decsta
  scenario_coex_ac_be
  scenario_coex_ac_be_dualsta
  scenario_coex_ac_be_quadsta
  scenario_coex_ac_be_decsta
  scenario_coex_n_ax
  scenario_coex_n_ax_dualsta
  scenario_coex_n_ax_triplesta
  scenario_coex_n_ax_quadsta
  scenario_coex_n_ax_decsta
  scenario_coex_n_be
  scenario_coex_n_be_dualsta
  scenario_coex_n_be_quadsta
  scenario_coex_n_be_decsta
  scenario_coex_ax_3sta
  scenario_coex_ax_5sta
  scenario_coex_ax_11sta
  scenario_coex_ax_2sta
  scenario_coex_be_ax
  scenario_coex_be_ax_dualsta
  scenario_coex_be_ax_quadsta
  scenario_coex_be_ax_decsta
  scenario_coex_be_2sta
  scenario_coex_be_3sta
  scenario_coex_be_5sta
  scenario_coex_be_11sta
  scenario_single_ax
  scenario_single_be
)

echo "Launching ${#SCENARIOS[@]} simulations..."

for scenario in "${SCENARIOS[@]}"; do
  log_file="$LOG_DIR/${scenario}.log"
  echo "  -> $scenario (log: $log_file)"
  (
    cd "$PROJECT_ROOT"
    ./ns3 run "$scenario" -- \
      --beMaxAmpdu="$BE_MAX_AMPDU" \
      --simulationTime="$SIMULATION_TIME" \
      --clientInterval="$CLIENT_INTERVAL"
  ) >"$log_file" 2>&1 &
done
wait

echo "All simulations finished. Logs available under $LOG_DIR "
