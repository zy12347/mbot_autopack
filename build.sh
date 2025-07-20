# 检查是否提供了包名参数
if [ -z "$1" ]; then
    echo "错误：请提供要编译的包名"
    echo "用法：$0 <package_name>"
    exit 1
fi

PACKAGE_ARGS=""
for pkg in "$@"; do
    PACKAGE_ARGS+=" $pkg"
done


# 使用传入的参数作为包名
colcon build --packages-select $PACKAGE_ARGS

# colcon build --packages-select mbot_pkg