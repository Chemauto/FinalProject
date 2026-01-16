{
unset http_proxy
unset https_proxy
unset HTTP_PROXY
unset HTTPS_PROXY
unset ALL_PROXY
unset all_proxy
unset no_proxy
unset NO_PROXY
}
#代理出现问题的时候使用，应该有更好的解决方法

# printenv | grep -i proxy
# 检查代理是否还是存在的