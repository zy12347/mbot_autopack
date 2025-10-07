# 检查是否提供了包名参数
if [ -z "$1" ]; then
    echo "错误：请提供要编译的包名"
    echo "用法：$0 <package_name>"
    exit 1
fi
rebuild=false          # 标记是否需要重建（删除缓存）
PACKAGE_ARGS=""        # 最终传给 colcon build 的包列表
ALL_PACKAGES="slam mbot_pkg mbot_interface control navigation"
install_only=false    # 标记是否仅安装（不编译）
# 解析命令行参数
for arg in "$@"; do
    case "$arg" in
        -r)
            rebuild=true
            ;;
        -a)
            PACKAGE_ARGS="$ALL_PACKAGES"
            ;;
        -i)
            install_only=true  # 检测到-i参数，仅执行安装
            ;;
        *)
            if [ -n "$arg" ]; then
                PACKAGE_ARGS+=" $arg"
            fi
            ;;
    esac
done

# 清理PACKAGE_ARGS中的多余空格
PACKAGE_ARGS=$(echo "$PACKAGE_ARGS" | xargs)

if [ "$rebuild" = true ]; then
    echo "=== 开始清理指定包的缓存 ==="
    for pkg in $PACKAGE_ARGS; do
        # 定义各缓存路径
        build_dir="./build/$pkg"
        install_dir="./install/$pkg"
        log_dir="./log/latest/$pkg"
        
        # 仅删除存在的目录
        if [ -d "$build_dir" ]; then
            echo "删除 $build_dir"
            rm -rf "$build_dir"
        fi
        if [ -d "$install_dir" ]; then
            echo "删除 $install_dir"
            rm -rf "$install_dir"
        fi
        if [ -d "$log_dir" ]; then
            echo "删除 $log_dir"
            rm -rf "$log_dir"
        fi
    done
    echo "=== 缓存清理完成 ==="
fi

# 使用传入的参数作为包名

echo "=== 编译并安装包：$PACKAGE_ARGS ==="
# 正常编译+安装
colcon build --packages-select $PACKAGE_ARGS
# colcon build --packages-select $PACKAGE_ARGS
if [ $? -eq 0 ]; then
    echo "=== 编译成功，更新环境变量 ==="
    source ./install/setup.bash
else
    echo "错误：编译失败！"
    exit 1
fi

# colcon build --packages-select mbot_pkg