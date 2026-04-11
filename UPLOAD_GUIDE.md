# GitHub 上传步骤指南

## 准备工作

### 1. 安装 Git（如果尚未安装）
下载并安装 Git for Windows: https://git-scm.com/download/win

### 2. 配置 Git 用户信息
打开 Git Bash 或 PowerShell，执行以下命令：
```bash
git config --global user.email "your-email@example.com"
git config --global user.name "Your Name"
```

---

## 上传步骤

### 方法一：使用 Git 命令行

#### 步骤 1：打开项目目录
```bash
cd "C:\Users\ttbstt\Desktop\stm32\HAL\sanlun_robomaster"
```

#### 步骤 2：初始化 Git 仓库（如果尚未初始化）
```bash
git init
```

#### 步骤 3：配置远程仓库
```bash
git remote add origin https://github.com/ttbstt3-cell/SANLUN-ROBOMASTER.git
```

#### 步骤 4：添加文件到暂存区
```bash
git add .
```

#### 步骤 5：提交更改
```bash
git commit -m "Add RoboMaster C板 SBUS 遥控车控制代码及技术说明文档"
```

#### 步骤 6：推送到 GitHub
```bash
git push -u origin main
```

**注意**：如果仓库中已有内容，可能需要先 pull：
```bash
git pull origin main --allow-unrelated-histories
git push -u origin main
```

---

### 方法二：使用 GitHub Desktop

#### 步骤 1：下载并安装 GitHub Desktop
https://desktop.github.com/

#### 步骤 2：打开 GitHub Desktop 并登录

#### 步骤 3：添加本地仓库
- 点击 "File" → "Add Local Repository"
- 选择项目目录：`C:\Users\ttbstt\Desktop\stm32\HAL\sanlun_robomaster`

#### 步骤 4：填写提交信息
- 在左下角的 "Summary" 框中输入：`Add RoboMaster C板 SBUS 遥控车控制代码及技术说明文档`

#### 步骤 5：点击 "Commit to main"

#### 步骤 6：点击 "Push origin" 推送到 GitHub

---

### 方法三：使用 VS Code（推荐）

#### 步骤 1：在 VS Code 中打开项目目录

#### 步骤 2：点击左侧的 Git 图标

#### 步骤 3：点击 "初始化仓库"（如果是第一次）

#### 步骤 4：在 "更改" 区域点击 "+" 暂存所有更改

#### 步骤 5：在上方的输入框中输入提交信息

#### 步骤 6：点击 "✓" 提交

#### 步骤 7：点击 "同步更改" 推送到 GitHub
- 首次同步时会提示登录 GitHub

---

## 文件说明

本次上传包含以下文件：

| 文件 | 说明 |
|------|------|
| `Core/Src/main.c` | 主程序，包含双模式控制逻辑 |
| `Core/Src/servo.c` | 舵机 PWM 控制实现 |
| `Core/Src/sbus.c` | SBUS 协议解析实现 |
| `Core/Inc/servo.h` | 舵机控制头文件 |
| `Core/Inc/sbus.h` | SBUS 协议头文件 |
| `README.md` | 快速入门指南 |
| `DOCUMENTATION.md` | **技术说明文档（新增）** |

---

## 常见问题

### Q: 推送时提示需要登录
**A**: 在弹出的窗口中登录您的 GitHub 账户

### Q: 提示 "refusing to merge unrelated histories"
**A**: 执行以下命令：
```bash
git pull origin main --allow-unrelated-histories
```

### Q: 推送被拒绝 "denied to username"
**A**: 检查是否有仓库的推送权限，或者使用个人访问令牌（Personal Access Token）

### Q: 网络连接失败
**A**: 
- 检查网络连接
- 如果使用代理，配置 Git 代理：
```bash
git config --global http.proxy http://proxy.example.com:8080
git config --global https.proxy https://proxy.example.com:8080
```

---

## 上传完成后

1. 访问 https://github.com/ttbstt3-cell/SANLUN-ROBOMASTER 确认文件已上传
2. 确认 `DOCUMENTATION.md` 已包含在仓库中

---

如有任何问题，请随时询问！
