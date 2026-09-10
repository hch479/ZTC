# ZTC：本次 GitHub 整理与 VS Code 推送、拉取方法

日期：2026-09-10。

## 1. 仓库与本地目录

GitHub 地址：[hch479/ZTC](https://github.com/hch479/ZTC)。该仓库当前为公开仓库。

本次操作的本地项目是：

```text
C:\Users\xixun\Desktop\C30D_Development_Handoff_20260909
```

请在 VS Code 中打开这个文件夹，确认能看到 `common`、`firmware`、`keil_project`、`ros2_ws` 等目录。另一个 `Documents\ChatGPT\ZTC` 目录目前存放会话报告和编译检查副本，不是本次上传的源码仓库。

使用分支 `main`，远端别名 `origin`，远端地址为 `https://github.com/hch479/ZTC.git`。

## 2. 本次怎样整理

检查时，本地已经执行过 Git 初始化，分支为 `main`，暂存了 926 个文件，但没有本地提交，也没有关联远端。暂存区包含设备备份、原厂资料和编译缓存。

Git 实际获取的远端已有一个 `Initial commit`，提交号 `c197325`，只含一行 `# ZTC` 的 README。本次以这个提交为基础继续提交项目，保留远端原始历史。

### 保留在 Git 的内容

- `common/`、`firmware/`：协议与控制核心。
- `keil_project/`：串口和保留的 CAN 工程，包括源码、启动文件、FreeRTOS、芯片库、`.uvprojx` 和 `.uvoptx` 配置。
- `ros2_ws/src/`：ROS 程序、参数、启动文件和脚本。
- `tests/`、`CMakeLists.txt`：离线测试和构建配置。
- `docs/`、README、LICENSE、AGENTS、交接说明及工具脚本。

### 仅保留在本地的内容

| 内容 | 为什么不上传 |
|---|---|
| `hardware_backups/` | 属于具体设备的 Flash 和选项字节备份 |
| `historical_builds/` | 历史固件与构建日志，开发源码已经保留 |
| `reference_materials/` | 原厂手册、原理图等资料，不属于本次源码发布范围 |
| `OBJ*/`、`.o`、`.axf`、`.hex`、`.map` 等 | 编译产生，可重新生成 |
| `build/`、`install/`、`log/` | ROS/CMake 构建输出 |
| Python 缓存和虚拟环境 | 机器本地生成内容 |
| `.uvguix.*`、`.vscode/` 等 | 当前电脑的编辑器布局和个人配置 |
| `SHA256SUMS.csv` | 原交接包的完整校验清单，与精简后的 Git 仓库范围不同 |

这些文件没有从电脑删除，只是由根目录 `.gitignore` 排除。重要设备备份仍需另外保存；仅克隆 GitHub 仓库不会恢复这些文件。

完整 Keil 工程里的厂商 C/H 库和启动文件仍然需要保留，否则换电脑可能缺少构建依赖。不能把所有“不是自己写的代码”都当成无用文件删除。

还添加了 `.gitattributes`，让 Linux shell 脚本在 Windows 检出时保持 LF 换行，批处理保持 CRLF，降低跨电脑开发的换行问题。

## 3. 本次 Git 操作逻辑

执行顺序如下：

1. 检查本地暂存区、分支、用户信息及远端历史。
2. 添加远端 `origin`，获取远端 `main`。
3. 备份原暂存区索引，将本地分支接到远端初始提交，保留工作目录中的项目文件。
4. 补齐 `.gitignore`，按新的规则重新暂存源码与文档。
5. 检查暂存清单，确认没有被忽略的备份或构建产物仍混在提交中，并检查常见凭据模式；检查未发现匹配，但不等于全面安全审计。
6. 提交整理后的项目，再推送 `main` 并设置上游关联。
7. 用远端提交号和本地提交号比较，确认推送是否完成；最终结果以随本说明交付的本次推送结果记录为准。

本次没有改动控制算法，也没有重新安装编译环境。此前本机完成过串口固件构建和核心测试；本次是 Git 整理，不把旧验证描述为本次重新测试。

这里有一个容易踩的坑：**`.gitignore` 对已经暂存或跟踪的文件不会自动生效。** 因此本次不只是写忽略规则，还重新整理了暂存区。

## 4. 先分清四个动作

| 动作 | 作用 |
|---|---|
| 保存 Save | 把编辑器内容写到本地文件 |
| 暂存 Stage | 选择哪些修改放入下一次提交 |
| 提交 Commit | 在本地 Git 历史里保存一个版本 |
| 推送 Push | 把本地提交上传到 GitHub |

所以，点了“提交”不表示 GitHub 已经更新，还要“推送”。反过来，“拉取 Pull”用于把远端提交取回并整合到当前分支。

## 5. 以后怎样在 VS Code 推送

按下面顺序操作：

1. 打开上述桌面项目，确认左下角分支为 `main`。
2. 修改代码，按 `Ctrl+S` 保存。
3. 按 `Ctrl+Shift+G` 打开“源代码管理”。
4. 点击变更文件查看差异，核对本次到底改了哪些内容。
5. 点击需要提交的文件右侧 `+`，加入“暂存的更改”。
6. 输入清楚的提交说明，例如 `完善 IMU 单位转换注释` 或 `增加融合断流测试`。
7. 点击“提交（Commit）”。
8. 在源代码管理的 `…` 菜单选择“推送（Push）”；也可在命令面板搜索 `Git: Push`。
9. 打开 GitHub 仓库，核对最新提交说明和修改文件。

VS Code 的“同步更改（Sync Changes）”会先拉取，再推送。刚开始建议分别使用 Pull 和 Push，便于知道当前执行的是哪个动作。[VS Code 官方快速入门](https://code.visualstudio.com/docs/sourcecontrol/quickstart)

以后不需要重复点击“初始化仓库”，也不需要再用“发布到 GitHub”创建同名仓库。

## 6. 以后怎样拉取

建议每次开始改代码前，先确认本地没有未处理修改，然后拉取：

1. 打开“源代码管理”。
2. 确认当前分支是 `main`。
3. 在 `…` 菜单选择“拉取（Pull）”，或命令面板搜索 `Git: Pull`。
4. 等待完成后再开始编辑。

本仓库设置了 `pull.ff=only`：远端只是比本地更新时，可以直接前进；如果两台电脑各自产生了不同提交，拉取会停下来让你处理，不会自动生成一个不清楚来源的合并提交。

“获取（Fetch）”只更新你对远端提交的了解，不直接修改当前工作文件；“拉取（Pull）”还会尝试整合远端提交。[VS Code 源代码管理说明](https://code.visualstudio.com/docs/sourcecontrol/overview)

如果本地有尚未提交的修改，先选择：完成后提交，或者暂存储藏（Stash）再拉取。拉取后恢复 Stash 也可能冲突。不要为了让拉取成功就随意点击“放弃更改”。

## 7. 两台电脑轮流开发

推荐形成这个习惯：

```text
电脑 A：拉取 → 修改 → 测试 → 提交 → 推送
电脑 B：拉取 → 修改 → 测试 → 提交 → 推送
切回 A：先拉取，再继续改
```

在一台新的电脑上，可以按 `Ctrl+Shift+P`，执行 `Git: Clone`，输入 `https://github.com/hch479/ZTC.git`，选择保存位置并打开克隆后的目录。

另一台旧电脑原有的项目可能有未上传修改，首次切换时应在新目录克隆，再比较原目录。不要直接覆盖原项目。

GitHub 同步源码，不同步编译器、Python 安装、ROS 环境，也不包含被忽略的备份。因此另一台电脑仍需配置自己的工具路径。

## 8. 常见问题

### 推送失败：远端有你没有的提交

先查看历史并获取远端。若只是本地落后，拉取后再推送；若双方都新增了提交，需要合并或 rebase。遇到冲突时，应根据实际代码含义决定保留什么，不能一律选“当前”或“传入”。处理完成、测试并提交后再推送。

不要用 Force Push 来处理普通同步问题，它可能覆盖另一台电脑已经上传的提交。

### GitHub 连接失败

本机 Git 直连曾出现 `Connection was reset`，使用现有代理 `127.0.0.1:7897` 后能够获取远端。因此只在这个本地仓库中配置了 GitHub 代理：

```powershell
git config --local http.https://github.com/.proxy http://127.0.0.1:7897
```

该设置位于本地 `.git/config`，不会随源码上传。使用此设置时，本机代理需要运行。以后不需要代理时，在这个仓库的终端执行：

```powershell
git config --local --unset http.https://github.com/.proxy
```

另一台电脑的代理端口可能不同，不应直接照搬。

### 登录提示

若 VS Code 或 Git Credential Manager 要求登录 GitHub，按其浏览器授权提示登录自己的账号。不要把密码或访问令牌写进源码、README 或带密码的远端地址。

### 文件为什么不显示在变更列表

先检查 `.gitignore`。Keil 生成的新 HEX、OBJ 缓存和备份目录不显示是预期行为；源码 `.c`、`.h`、`.py` 和项目配置仍应显示。

## 9. 对应的终端命令

日常操作可以用 VS Code 界面，也可在项目终端执行：

```powershell
# 开始工作前，先检查状态并拉取
git status
git pull --ff-only

# 修改后查看内容
git diff

# 暂存：明确选择本次修改的文件
git add ros2_ws/src/chassis_can_control/chassis_can_control/wheel_odometry.py

# 提交前查看暂存差异，然后提交、推送
git diff --cached
git commit -m "说明本次修改内容"
git push
```

检查是否同步：

```powershell
git status -sb
git log -3 --oneline
git remote -v
```

正常同步且没有本地修改时，状态应只显示 `main...origin/main`，不带 ahead/behind，也没有变更文件列表。
