# Scripts 使用说明

本目录包含导航任务自动评测与结果汇总脚本。

## 自动评测

```bash
python scripts/evaluate_navigation.py \
  --target-json eval_targets/navigation_targets.json \
  --scenes 0 1 2 3 4 \
  --trials 10 \
  --instruction "导航到目标点" \
  --threshold 0.3
```

常用参数：
- `--scenes`：测试场景编号 `0~4`；`--trials`：每场景重复次数。
- `--target-json`：固定真值目标；`--threshold`：最终 XY 误差阈值。
- `--restart-each-trial`：每次实验都重启 IsaacLab。

评测输出位于 `eval_results/`，包括 `.json` 和 `.md` 报告。

## 报告字段

- `duration_sec`：任务总耗时；`planning_duration_sec`：上层规划耗时。
- `execution_duration_sec`：技能执行耗时；`planner_calls`：规划调用次数。

## 按场景汇总

```bash
python scripts/summarize_navigation_eval.py \
  eval_results/navigation_eval_xxx.md \
  --prefix navigation_scene_summary
```

输出 `.json`、`.csv` 和 `.png` 文件到 `eval_results/`。

```bash
python scripts/summarize_navigation_eval.py report.md --planning-times 30,40,45,43,44
```

最后一条命令用于旧报告没有规划时间字段时手动指定规划耗时。
