# USB2CAN_motor —— 协作工作流接入

> 本文件位于项目代码根目录，会让在本项目工作的 agent 自动加载中央规则和本项目状态。

## 必做：自动加载规则 + 本项目状态
@/home/toe/prompt/RULES.md
@/home/toe/USB2CAN_motor/CONTEXT.md

完整流程手册见 `/home/toe/prompt/PROMPT.md`（新项目初始化 / 不确定流程时按需查阅）。
本项目的 `design.md` / `tasks/` 在 `/home/toe/USB2CAN_motor/`，按需读取。

## 红线：规则只读，状态才可写
- `/home/toe/prompt/RULES.md`、`/home/toe/prompt/PROMPT.md`、`CLAUDE.md`、`templates/` 是**只读规则/模板**，禁止修改。
- **只允许写本项目自己的状态**：`/home/toe/USB2CAN_motor/CONTEXT.md`、`design.md`、`tasks/`、`USAGE.md`。
- **绝不读/写别的项目**状态，避免串味。
- **代码写在本项目代码根目录**，不写进 `/home/toe/prompt`。
