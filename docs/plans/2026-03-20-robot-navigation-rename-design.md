# Robot Navigation Module Rename Design

## Goal

将 `Robot_Module/module/navigation_demo.py` 重命名为 `Robot_Module/module/navigation.py`，并同步更新模块导入、日志命名、运行命令和 `Robot_Module/README.md`，不改变任何技能行为。

## Scope

本次只做命名和文档整理：

1. 文件名从 `navigation_demo.py` 改为 `navigation.py`
2. 更新所有直接引用该路径的代码和文档
3. 将 `Robot_Module/README.md` 更新为当前实现说明，并控制在 60 行内

不修改：

- `walk/climb/push_box/way_select` 的工具名
- 技能参数
- 20 秒超时行为
- FastMCP 注册方式

## Design

### 1. 文件迁移

`navigation_demo.py` 当前已经不只是临时试验文件，它承载的是 `Robot_Module` 里实际使用的四足导航技能。

因此直接重命名为 `navigation.py` 更准确：

- 模块名更稳定
- 代码路径和 README 更一致
- 后续扩展时不会被 `demo` 命名误导

### 2. 导入与调用点

当前主要调用点是：

- `Robot_Module/skill.py`
- `Robot_Module/module/__init__.py`
- `Robot_Module/README.md`
- 模块内部日志字符串

改动后：

- `skill.py` 从 `module.navigation` 导入 `register_tools`
- 注册变量名改成 `navigation_tools`
- `module/__init__.py` 的模块说明更新为 `navigation.py`
- `Robot_Module/README.md` 中的路径和运行命令全部更新为 `navigation.py`

### 3. 日志与可读性

当前模块内部仍打印：

- `[navigation_demo.py:register_tools] ...`

这次一并改成：

- `[navigation.py:register_tools] ...`

这样终端输出和真实文件路径保持一致。

### 4. Testing

因为这是路径重命名，最有价值的回归测试不是功能逻辑，而是：

1. `Robot_Module.skill` 能继续成功导入导航模块
2. 新路径 `Robot_Module.module.navigation` 存在 `register_tools`
3. 旧路径引用不再残留在 `Robot_Module` README 和核心导入中

这次会新增一个最小测试文件覆盖这些点。

### 5. README

`Robot_Module/README.md` 目前存在两个问题：

- 路径还是旧的 `navigation_demo.py`
- 文档内容还是旧版描述

这次 README 会收敛成一份当前实现说明，覆盖：

- `Robot_Module` 的职责
- 已注册的导航模块
- 对外 4 个技能
- 20 秒反馈超时
- 当前运行命令

总行数控制在 60 行内。
