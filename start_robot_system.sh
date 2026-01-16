#!/bin/bash
# FinalProject - 主启动脚本
# 使用简化版架构（无 ROS2）

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# 项目根目录
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

# 加载 .env 文件 (如果存在)
if [ -f ".env" ]; then
    echo -e "${GREEN}加载 .env 文件...${NC}"
    export $(cat .env | grep -v '^#' | xargs)
fi

echo -e "${BLUE}========================================${NC}"
echo -e "${BLUE}FinalProject Robot Control System${NC}"
echo -e "${BLUE}简化版架构 (无 ROS2)${NC}"
echo -e "${BLUE}========================================${NC}"
echo ""

# 检查 Python 环境
if ! command -v python3 &> /dev/null; then
    echo -e "${RED}错误: 未找到 python3${NC}"
    exit 1
fi

# 检查 API Key
if [ -z "$Test_API_KEY" ]; then
    echo -e "${YELLOW}警告: 未设置 Test_API_KEY 环境变量${NC}"
    echo -e "请设置: export Test_API_KEY=your_api_key_here"
    echo ""
fi

echo -e "${GREEN}启动方式选择：${NC}"
echo -e "  ${YELLOW}1${NC}. 自动启动（两个终端窗口）"
echo -e "  ${YELLOW}2${NC}. 手动启动（显示命令）"
echo ""
read -p "请选择 (1/2): " choice

if [ "$choice" == "1" ]; then
    echo ""
    echo -e "${GREEN}正在启动系统...${NC}"
    echo ""

    # 终端 1: 仿真器（后台运行）
    cd "$SCRIPT_DIR"
    export PYTHONPATH="$SCRIPT_DIR:$PYTHONPATH"
    python3 Sim_Module/sim2d/simulator.py &
    SIM_PID=$!
    echo -e "${GREEN}✓${NC} 仿真器已启动 (PID: $SIM_PID)"

    # 等待仿真器启动
    sleep 2

    # 终端 2: 交互界面（前台运行）
    echo -e "${GREEN}✓${NC} 启动交互界面..."
    echo ""
    python3 Interactive_Module/interactive.py

    # 清理：当交互界面退出时，也关闭仿真器
    kill $SIM_PID 2>/dev/null
    echo ""
    echo -e "${GREEN}系统已关闭${NC}"

elif [ "$choice" == "2" ]; then
    echo ""
    echo -e "${BLUE}========================================${NC}"
    echo -e "${BLUE}手动启动指南${NC}"
    echo -e "${BLUE}========================================${NC}"
    echo ""
    echo -e "${GREEN}请打开两个终端窗口，分别执行：${NC}"
    echo ""
    echo -e "${YELLOW}终端 1 - 仿真器:${NC}"
    echo "  cd $SCRIPT_DIR"
    echo "  export PYTHONPATH=$SCRIPT_DIR:\$PYTHONPATH"
    echo "  python3 Sim_Module/sim2d/simulator.py"
    echo ""
    echo -e "${YELLOW}终端 2 - 交互界面:${NC}"
    echo "  cd $SCRIPT_DIR"
    echo "  export PYTHONPATH=$SCRIPT_DIR:\$PYTHONPATH"
    echo "  export Test_API_KEY=your_api_key_here"
    echo "  python3 Interactive_Module/interactive.py"
    echo ""
    echo -e "${BLUE}========================================${NC}"
    echo ""

else
    echo -e "${RED}无效选择${NC}"
    exit 1
fi
