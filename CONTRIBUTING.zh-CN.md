[English](CONTRIBUTING.md) | 中文

# 贡献指南

几条小规则，由评审执行。

注释、日志字符串、提交信息和文档使用英文。CJK 扫描是评审的一部分：
`grep -rlP '[\x{4e00}-\x{9fff}]' rt-host client docs` 应该只列出 `rt-host/README.zh-CN.md` 中
“Known gaps”下仍然点名的文件，而且这个列表只应该越来越短。

文档是双语的：每个 .md 都有一个 .zh-CN.md 孪生文件，第一行是语言切换行；两边要保持同步。代码注释、
docstring 和日志字符串只用英文。

新代码中不要出现绝对的家目录路径。用变量（`FRANKA_ROOT`、`$HOME`、`$(dirname "$0")`）代替
`/home/<user>/...`。现有的 `rt-host/` 脚本仍带有 `/home/rongxuan_zhou/franka`，已列为已知缺口；不要
再增加。

对每个改动过的 shell 脚本运行 `bash -n` 和 `shellcheck`，对每个改动过的 Python 文件运行
`python3 -m py_compile`。凡是在主机上或客户端 mirror 守护进程里运行的代码，只能用标准库。

接口已冻结。对 `docs/INTERFACE.zh-CN.md` 中的地址、数据报格式、文件名、速率、动词或退出码的任何改动，
都会升级协议标签（`FRST1` 到 `FRST2`），并且必须在同一个 PR 中协调修改两半。

凡是会移动机器人的改动，都必须重新执行 `docs/ACCEPTANCE.zh-CN.md` 中对应的清单步骤并记录数据，
否则不能合入。

**绝不提交**私钥、`keys/`、数据集、录制文件或编译后的二进制。`.gitignore` 覆盖了常见的名字，但提交前
还是先看一眼 `git status`。
