# Root README Condense Design

## Goal

将项目根目录 `README.md` 压缩为一份不超过 80 行的当前实现速览文档。

## Scope

这次只更新根 README，不修改任何代码、提示词或模块行为。

## Design

README 保留四类信息：

1. 项目定位和当前 4 个技能
2. 当前主链路和关键约束
3. `object_facts.json` 输入方式
4. 最常用运行入口和核心模块路径

README 删掉长篇展开内容，包括：

- 详细字段逐项解释
- 参数计算的长段推导
- 大段模块说明
- 重复性的“为什么这么做”背景

最终文档要明确当前真实行为：

- 主链路是 `user_input -> VLM -> scene_facts -> highlevel -> parameter calculation -> lowlevel -> tool result`
- `object_facts` 优先于 VLM
- 失败即停止，不再重规划
- 执行反馈默认等待 20 秒
- 导航模块路径为 `Robot_Module/module/navigation.py`

## Verification

这次不需要代码测试，验证标准只有两个：

- `README.md` 行数 `<= 80`
- 文案与当前实现一致
