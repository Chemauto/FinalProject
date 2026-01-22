#!/bin/bash

# WebUI 一键启动脚本

set -e

# 颜色定义
GREEN='\033[0;32m'
BLUE='\033[0;34m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m' # No Color

# 打印带颜色的消息
print_info() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

print_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# 检查命令是否存在
command_exists() {
    command -v "$1" >/dev/null 2>&1
}

# 获取脚本目录
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

print_info "Robot WebUI 启动脚本"
echo "========================================"

# 检查 .env 文件
if [ ! -f "../.env" ]; then
    print_warning "未找到 .env 文件"
    print_info "后端将尝试从环境变量加载 API Key"
else
    print_success "找到 .env 文件"
fi

# 检查依赖
print_info "检查依赖..."

if ! command_exists python3; then
    print_error "未找到 python3"
    exit 1
fi

if ! command_exists npm; then
    print_error "未找到 npm"
    exit 1
fi

print_success "依赖检查完成"

# 安装后端依赖
print_info "检查后端依赖..."
cd "$SCRIPT_DIR/backend"
if [ ! -d "venv" ]; then
    print_info "创建 Python 虚拟环境..."
    python3 -m venv venv
fi

source venv/bin/activate
pip install -q -r requirements.txt
print_success "后端依赖就绪"

# 安装前端依赖
print_info "检查前端依赖..."
cd "$SCRIPT_DIR/frontend"
if [ ! -d "node_modules" ]; then
    print_info "安装前端依赖（首次运行可能需要几分钟）..."
    npm install --silent
fi
print_success "前端依赖就绪"

# 构建前端
print_info "构建前端..."
npm run build
print_success "前端构建完成"

# 启动后端服务器
cd "$SCRIPT_DIR/backend"
print_info "启动后端服务器..."
echo ""
print_info "访问地址: http://localhost:5000"
print_info "按 Ctrl+C 停止服务器"
echo "========================================"
echo ""

# 激活虚拟环境并启动
source venv/bin/activate
python app.py
