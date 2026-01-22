#!/bin/bash
# 追击功能测试启动脚本 (Linux/Mac)

set -e

# 颜色定义
GREEN='\033[0;32m'
BLUE='\033[0;34m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m'

# 获取脚本目录
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
cd "$SCRIPT_DIR/.."

print_info() {
    echo -e "${BLUE}[信息]${NC} $1"
}

print_success() {
    echo -e "${GREEN}[成功]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[警告]${NC} $1"
}

print_error() {
    echo -e "${RED}[错误]${NC} $1"
}

echo "========================================"
echo "追击功能测试 - 启动脚本"
echo "========================================"
echo ""

# 检查 Python
if ! command -v python3 &> /dev/null; then
    print_error "未找到 python3"
    exit 1
fi

print_success "找到 Python: $(python3 --version)"
print_info "项目目录: $(pwd)"
echo ""

# 检查 .env
if [ ! -f ".env" ]; then
    print_warning "未找到 .env 文件"
    print_info "请确保已设置 Test_API_KEY"
    echo ""
else
    print_success "找到 .env 文件"
    echo ""
fi

echo "========================================"
echo "请选择启动模式:"
echo "========================================"
echo ""
echo "1. 仅启动仿真器"
echo "2. 仅启动测试程序"
echo "3. 同时启动（两个窗口）"
echo "4. 退出"
echo ""

read -p "请输入选择 (1-4): " choice

case $choice in
    1)
        echo ""
        print_info "启动仿真器..."
        echo ""
        python3 Test_Module/enhanced_simulator.py
        ;;
    2)
        echo ""
        print_info "启动测试程序..."
        echo ""
        python3 Test_Module/test_chase.py
        ;;
    3)
        echo ""
        print_info "启动仿真器和测试程序..."
        echo ""

        # 检查是否支持多终端
        if command -v gnome-terminal &> /dev/null; then
            gnome-terminal -- bash -c "python3 Test_Module/enhanced_simulator.py" &
            sleep 2
            gnome-terminal -- bash -c "python3 Test_Module/test_chase.py" &
        elif command -v xterm &> /dev/null; then
            xterm -e "python3 Test_Module/enhanced_simulator.py" &
            sleep 2
            xterm -e "python3 Test_Module/test_chase.py" &
        else
            print_warning "无法自动打开多个终端"
            print_info "请手动运行："
            print_info "  终端1: python3 Test_Module/enhanced_simulator.py"
            print_info "  终端2: python3 Test_Module/test_chase.py"
            exit 0
        fi

        echo ""
        print_success "两个窗口已打开"
        print_info "先在仿真器中按 'R' 生成敌人"
        print_info "然后在测试程序中输入追击命令"
        ;;
    4)
        exit 0
        ;;
    *)
        print_error "无效选择"
        exit 1
        ;;
esac
