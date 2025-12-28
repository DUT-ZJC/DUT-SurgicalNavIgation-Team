#pragma once
#include "spdlog/spdlog.h"
#include "spdlog/sinks/stdout_color_sinks.h" // 控制台输出
#include "spdlog/sinks/basic_file_sink.h"    // 文件输出

inline bool logFlag = true;

inline void InitLogger() {
    // 1. 创建控制台 Sink (带颜色)
    auto console_sink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
    console_sink->set_level(spdlog::level::info); // 控制台只打印 INFO 及以上
    console_sink->set_pattern("[%^%l%$] %v");    // 格式: [INFO] 消息内容 (%^%$是颜色)

    // 2. 创建文件 Sink (文件名: logs/stereo.txt, true表示每次清空重写)
    auto file_sink = std::make_shared<spdlog::sinks::basic_file_sink_mt>("logs/stereo_log.txt", true);
    file_sink->set_level(spdlog::level::trace);   // 文件记录所有细节
    file_sink->set_pattern("[%Y-%m-%d %H:%M:%S.%e] [%l] %v"); // 格式: [时间] [级别] 内容

    // 3. 组合这两个 Sink
    std::vector<spdlog::sink_ptr> sinks{ console_sink, file_sink };
    auto logger = std::make_shared<spdlog::logger>("multi_sink", sinks.begin(), sinks.end());

    // 4. 注册并设置为默认
    spdlog::register_logger(logger);
    spdlog::set_default_logger(logger);

    // 5. 设置全局刷新策略 (每当遇到 error 级别立即写入文件，防止崩溃时日志丢失)
    spdlog::flush_on(spdlog::level::err);
}