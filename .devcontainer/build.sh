#!/bin/bash

# =============================================
# 自动获取Git分支名作为Docker镜像标签的构建脚本
# 支持分支名标签、主分支latest标签、私有仓库推送
# =============================================

set -e  # 遇到错误立即退出

# -------------------------------
# 颜色定义
# -------------------------------
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # 恢复默认颜色

# 日志函数
log_info() { echo -e "${BLUE}[INFO]${NC} $1"; }
log_success() { echo -e "${GREEN}[SUCCESS]${NC} $1"; }
log_warning() { echo -e "${YELLOW}[WARNING]${NC} $1"; }
log_error() { echo -e "${RED}[ERROR]${NC} $1"; }

# -------------------------------
# 配置参数（可根据需要修改）
# -------------------------------
APP_NAME="ai"                    # 应用名称
DEFAULT_REGISTRY="docker.io"            # 默认镜像仓库
DEFAULT_USERNAME="yutouxiaozhuan" # 你的DockerHub用户名
DOCKERFILE_PATH="."                      # Dockerfile 所在目录

# -------------------------------
# 使用说明
# -------------------------------
usage() {
    cat << EOF
用法: $0 [选项]

选项:
    -u, --username USERNAME    DockerHub 用户名 (默认: $DEFAULT_USERNAME)
    -r, --registry REGISTRY    镜像仓库地址 (默认: $DEFAULT_REGISTRY)
    -b, --branch BRANCH        指定分支名 (默认: 自动检测当前Git分支)
    -p, --push                  构建后推送到仓库
    -c, --cleanup               构建完成后清理临时镜像
    -h, --help                 显示此帮助信息

示例:
    $0 -u myusername -p              # 使用默认设置构建并推送
    $0 -b develop -p                 # 使用特定分支构建并推送
    $0 --cleanup                     # 构建后清理
EOF
}

# -------------------------------
# 参数解析
# -------------------------------
PUSH_IMAGE=true
CLEANUP_IMAGES=false
CUSTOM_BRANCH=""

while [[ $# -gt 0 ]]; do
    case $1 in
        -u|--username)
            DOCKER_USERNAME="$2"
            shift 2
            ;;
        -r|--registry)
            DOCKER_REGISTRY="$2"
            shift 2
            ;;
        -b|--branch)
            CUSTOM_BRANCH="$2"
            shift 2
            ;;
        -p|--push)
            PUSH_IMAGE=true
            shift
            ;;
        -c|--cleanup)
            CLEANUP_IMAGES=true
            shift
            ;;
        -h|--help)
            usage
            exit 0
            ;;
        *)
            log_error "未知参数: $1"
            usage
            exit 1
            ;;
    esac
done

# 设置默认值
DOCKER_USERNAME=${DOCKER_USERNAME:-$DEFAULT_USERNAME}
DOCKER_REGISTRY=${DOCKER_REGISTRY:-$DEFAULT_REGISTRY}

# -------------------------------
# 自动获取Git分支信息
# -------------------------------
get_git_branch() {
    if [[ -n "$CUSTOM_BRANCH" ]]; then
        echo "$CUSTOM_BRANCH"
    else
        # 方法1: 使用git rev-parse（兼容性最好）[1,7,8](@ref)
        local branch=$(git rev-parse --abbrev-ref HEAD 2>/dev/null)
        
        # 如果获取失败或处于分离头指针状态，使用提交哈希[1](@ref)
        if [[ $? -ne 0 || "$branch" == "HEAD" ]]; then
            branch="commit-$(git rev-parse --short HEAD 2>/dev/null || echo "unknown")"
        elif [[ -z "$branch" ]]; then
            branch="unknown-branch"
        fi
        
        echo "$branch"
    fi
}

# 清理分支名（Docker标签不允许特殊字符）[1](@ref)
clean_branch_name() {
    local branch_name="$1"
    # 将不允许的字符（如 /）替换为 -
    echo "$branch_name" | sed 's/[^a-zA-Z0-9._-]/-/g'
}

# -------------------------------
# 版本号生成
# -------------------------------
generate_version() {
    local branch_name=$(get_git_branch)
    local clean_branch=$(clean_branch_name "$branch_name")

    # 判断：如果是主分支，则使用 'latest'，否则使用清理后的分支名
    if [[ "$branch_name" == "main" || "$branch_name" == "master" ]]; then
        echo "latest"
    else
        echo "$clean_branch"
    fi
}

BRANCH_NAME=$(get_git_branch)
CLEAN_BRANCH_NAME=$(clean_branch_name "$BRANCH_NAME")
VERSION=$(generate_version)

FULL_IMAGE_NAME="${DOCKER_REGISTRY}/${DOCKER_USERNAME}/${APP_NAME}:${VERSION}"
LATEST_IMAGE_NAME="${DOCKER_REGISTRY}/${DOCKER_USERNAME}/${APP_NAME}:latest"
BRANCH_IMAGE_NAME="${DOCKER_REGISTRY}/${DOCKER_USERNAME}/${APP_NAME}:${CLEAN_BRANCH_NAME}"

# -------------------------------
# 前置检查
# -------------------------------
perform_checks() {
    log_info "执行前置检查..."
    
    # 检查是否在Git仓库中
    if ! git rev-parse --git-dir >/dev/null 2>&1 && [[ -z "$CUSTOM_BRANCH" ]]; then
        log_error "当前目录不是Git仓库，请使用 -b 参数指定分支名"
        exit 1
    fi
    
    # 检查 Docker 是否安装
    if ! command -v docker &> /dev/null; then
        log_error "Docker 未安装，请先安装 Docker"
        exit 1
    fi
    
    # 检查 Docker 守护进程是否运行
    if ! docker info >/dev/null 2>&1; then
        log_error "Docker 守护进程未运行，请启动 Docker"
        exit 1
    fi
    
    # 检查 Dockerfile 是否存在
    if [[ ! -f "${DOCKERFILE_PATH}/Dockerfile" ]]; then
        log_error "在 ${DOCKERFILE_PATH} 目录下未找到 Dockerfile"
        exit 1
    fi
    
    log_success "前置检查通过"
}

# -------------------------------
# 构建镜像
# -------------------------------
build_image() {
    log_info "开始构建 Docker 镜像..."
    log_info "Git 分支: ${BRANCH_NAME}"
    log_info "清理后分支名: ${CLEAN_BRANCH_NAME}"
    log_info "主镜像: ${FULL_IMAGE_NAME}"
    
    # 构建主镜像
    docker build \
        --file "${DOCKERFILE_PATH}/Dockerfile" \
        --tag "${FULL_IMAGE_NAME}" \
        "${DOCKERFILE_PATH}"
    
    # 为分支名创建额外标签（便于直接使用分支名拉取）
    docker tag "${FULL_IMAGE_NAME}" "${BRANCH_IMAGE_NAME}"
    
    # 如果是主分支，额外标记为latest[1](@ref)
    if [[ "$BRANCH_NAME" == "main" || "$BRANCH_NAME" == "master" ]]; then
        log_info "检测到主分支，同时标记为 'latest'"
        docker tag "${FULL_IMAGE_NAME}" "${LATEST_IMAGE_NAME}"
    fi
    
    log_success "镜像构建成功"
}

# -------------------------------
# 推送镜像到仓库
# -------------------------------
# -------------------------------
# 推送镜像到仓库
# -------------------------------
push_image() {
    if [[ "$PUSH_IMAGE" != true ]]; then
        log_info "跳过镜像推送（未使用 -p 参数）"
        return 0
    fi
    
    log_info "准备推送镜像到仓库..."
    
    # 从配置文件读取PAT
    local TOKEN_FILE="../.docker/token"
    if [[ ! -f "$TOKEN_FILE" ]]; then
        log_error "找不到Docker token文件: $TOKEN_FILE"
        log_error "请执行以下命令创建token文件:"
        log_error "echo '你的PAT令牌' > ../.docker/token && chmod 600 ~/.docker/token"
        exit 1
    fi
    
    local DOCKER_PAT=$(cat "$TOKEN_FILE" | tr -d '[:space:]')
    
    if [[ -z "$DOCKER_PAT" ]]; then
        log_error "Docker token文件为空"
        exit 1
    fi
    
    log_info "使用Personal Access Token自动登录Docker Hub..."
    echo "$DOCKER_PAT" | docker login --username "$DOCKER_USERNAME" --password-stdin >/dev/null 2>&1
    if [ $? -eq 0 ]; then
        log_success "Docker Hub自动登录成功"
    else
        log_error "Docker Hub自动登录失败，请检查PAT令牌是否正确"
        exit 1
    fi
    
    # 推送镜像
    log_info "推送镜像: ${FULL_IMAGE_NAME}"
    docker push "$FULL_IMAGE_NAME"
    
    log_info "推送分支标签: ${BRANCH_IMAGE_NAME}"
    docker push "$BRANCH_IMAGE_NAME"
    
    # 如果是主分支，推送latest标签
    if [[ "$BRANCH_NAME" == "main" || "$BRANCH_NAME" == "master" ]]; then
        log_info "推送最新标签: ${LATEST_IMAGE_NAME}"
        docker push "$LATEST_IMAGE_NAME"
    fi
    
    log_success "镜像推送完成"
}

# -------------------------------
# 清理临时镜像
# -------------------------------
cleanup() {
    if [[ "$CLEANUP_IMAGES" != true ]]; then
        return 0
    fi
    
    log_info "清理临时镜像..."
    docker system prune -f
    log_success "清理完成"
}

# -------------------------------
# 显示构建结果
# -------------------------------
show_results() {
    log_success "=== 构建结果 ==="
    echo -e "Git 分支: ${BLUE}${BRANCH_NAME}${NC}"
    echo -e "镜像版本: ${BLUE}${VERSION}${NC}"
    echo -e "主镜像: ${BLUE}${FULL_IMAGE_NAME}${NC}"
    echo -e "分支标签: ${BLUE}${BRANCH_IMAGE_NAME}${NC}"
    
    if [[ "$BRANCH_NAME" == "main" || "$BRANCH_NAME" == "master" ]]; then
        echo -e "最新标签: ${BLUE}${LATEST_IMAGE_NAME}${NC}"
    fi
    
    echo -e "构建时间: ${BLUE}$(date)${NC}"
    
    if [[ "$PUSH_IMAGE" == true ]]; then
        echo -e "推送状态: ${GREEN}已推送${NC}"
    else
        echo -e "推送状态: ${YELLOW}未推送${NC}"
    fi
    
    # 显示镜像信息
    log_info "生成的镜像:"
    docker images | grep "${DOCKER_REGISTRY}/${DOCKER_USERNAME}/${APP_NAME}" | head -5
}

# -------------------------------
# 主执行流程
# -------------------------------
main() {
    log_info "开始 Docker 镜像构建流程..."
    log_info "应用名称: $APP_NAME"
    log_info "仓库地址: $DOCKER_REGISTRY"
    log_info "用户名: $DOCKER_USERNAME"
    
    # 执行各个步骤
    perform_checks
    build_image
    push_image
    show_results
    cleanup
    
    log_success "Docker 镜像构建流程完成！"
}

# 捕获中断信号
trap 'log_error "脚本被用户中断"; exit 1' INT

# 执行主函数
main "$@"