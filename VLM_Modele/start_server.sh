#!/bin/bash
python -m vllm.entrypoints.openai.api_server \
  --model /home/dora/RoboBrain2.0/Model_3B \ #这里填写模型所在的位置
  --host 0.0.0.0 \
  --port 8000 \ #填写号端口号
  --served-model-name robobrain \ #注意这个参数在调用的时候也需要写对
  --gpu-memory-utilization 0.92 \
  --max-model-len 20000 \
  --max-num-seqs 256
