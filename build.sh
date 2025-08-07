# 检查是否提供了包名参数
if [ -z "$1" ]; then
    echo "错误：请提供要编译的包名"
    echo "用法：$0 <package_name>"
    exit 1
fi

# 检查是否有 -r 参数，如果有则删除 build、install、log 文件夹
if [[ " $@ " =~ " -r " ]]; then
    echo "检测到 -r 参数，正在删除 build、install、log 文件夹..."
    rm -rf build install log
    # 移除 -r 参数，防止后续 colcon build 误识别
    set -- "${@/-r/}"
fi


PACKAGE_ARGS=""
for pkg in "$@"; do
    PACKAGE_ARGS+=" $pkg"
done


# 使用传入的参数作为包名
colcon build --packages-select $PACKAGE_ARGS

# colcon build --packages-select mbot_pkg