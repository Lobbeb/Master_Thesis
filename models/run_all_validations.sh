#!/usr/bin/env bash

set -u

cd /home/ruben/halmstad_ws/models || exit 1

DATASETS_ROOT="/home/ruben/halmstad_ws/datasets/tmp_datasets"
RESULTS_ROOT="/home/ruben/halmstad_ws/models/results/batch_validation_logs"

mkdir -p "$RESULTS_ROOT"

models=(
  "baylands-leader-v1.pt"
  "baylands-leader-v3.pt"
  "baylands-leader-v4-2.pt"
  "baylands-leader-v5.pt"
  "baylands-leader-v6.pt"
)

subsets=(
  
  "rotundan"
)

summary_file="$RESULTS_ROOT/validation_summary.csv"
echo "dataset,model,status" > "$summary_file"

for subset in "${subsets[@]}"; do
  for version in {1..8}; do

    dataset_name="${subset}_v${version}"
    dataset_yaml="$DATASETS_ROOT/$dataset_name/dataset.yaml"

    if [ ! -f "$dataset_yaml" ]; then
      echo "Skipping missing dataset: $dataset_name"
      continue
    fi

    for model in "${models[@]}"; do
      echo
      echo "============================================================"
      echo "Validating dataset: $dataset_name"
      echo "Model:              $model"
      echo "============================================================"
      echo

      python3 -u model_validate.py \
        --dataset "$dataset_name" \
        --model "$model"

      status=${PIPESTATUS[0]}

      if [ "$status" -eq 0 ]; then
        echo "$dataset_name,$model,OK" >> "$summary_file"
      else
        echo "$dataset_name,$model,FAILED" >> "$summary_file"
        echo "Failed: $dataset_name with $model"
      fi

    done
  done
done

echo
echo "Finished validation batch."
echo "Summary:"
echo "$summary_file"