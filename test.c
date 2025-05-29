#define MAX(a, b) ((a) > (b) ? (a) : (b))

// 递归展开宏定义
#define ARG_MAX(...) ARG_MAX_IMPL(__VA_ARGS__)
#define ARG_MAX_IMPL(...) ARG_MAX_IMPL_(__VA_ARGS__)
#define ARG_MAX_IMPL_(x, ...) MAX(x, ARG_MAX_IMPL(__VA_ARGS__))
#define ARG_MAX_IMPL_(x) (x)  // 终止条件：单参数时返回自身

// 应用宏到具体数值序列
#define MAX_VALUE ARG_MAX( \
    8, 8, 8, 24, 8, 8, 8, 8, 8, 8, \
    8, 8, 8, 8, 8, 8, 16, 8, 8, 8, \
    8, 8, 8, 8, 8, 8, 8, 8, 8, 8, \
    16, 8, 8, 8, 0 \
)