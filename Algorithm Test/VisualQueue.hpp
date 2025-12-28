#pragma once
#include <deque> // 替换 queue
#include <mutex>
#include <condition_variable>
#include <opencv2/opencv.hpp>
#include <algorithm> // for std::clamp

class BrowsableQueue {
private:
    std::deque<cv::Mat> buffer; // 使用 deque 以支持下标访问
    std::mutex mtx;
    std::condition_variable cond;
    size_t max_size;

    // 游标位置： -1 表示“实时模式”(始终看最新)，>=0 表示“查看特定帧”
    int cursor_index = -1;

public:
    BrowsableQueue(size_t limit = 100) : max_size(limit) {}

    // --- 生产者：入队 ---
    void push(cv::Mat mat) {
        std::lock_guard<std::mutex> lock(mtx);

        // 丢帧策略：如果满了，移除最旧的（头部）
        if (buffer.size() >= max_size) {
            buffer.pop_front();

            // 【关键逻辑】如果删除了头部，且当前正在回放，
            // 游标索引需要减1，以保证它指向的还是原来的那张图
            if (cursor_index > 0) {
                cursor_index--;
            }
            else if (cursor_index == 0) {
                // 如果恰好在看最旧的那张，它被删了，那只能被迫看新的最旧张
                cursor_index = 0;
            }
        }

        buffer.push_back(std::move(mat));
        cond.notify_one();
    }

    // --- 消费者/可视化：获取当前要显示的图像 ---
    // 返回 true 表示获取成功，output 为图像
    bool get_display_image(cv::Mat& output) {
        std::lock_guard<std::mutex> lock(mtx);

        if (buffer.empty()) return false;

        // 确定要拿哪一张
        int target_idx;

        if (cursor_index == -1) {
            target_idx = buffer.size() - 1; // 实时模式：拿最后一张
        }
        else {
            // 回放模式：确保索引不越界
            target_idx = std::clamp(cursor_index, 0, (int)buffer.size() - 1);
        }

        // 这里必须用 clone 或者 copy，因为我们只是“偷看”，不能把图从队列里 move 走
        // 否则下次翻阅回来图就没了。
        output = buffer[target_idx].clone();
        return true;
    }

    // --- 控制逻辑：上一张 ---
    // 返回：当前是否处于回放模式
    bool move_prev() {
        std::lock_guard<std::mutex> lock(mtx);
        if (buffer.empty()) return false;

        // 如果当前是实时模式，先初始化游标到末尾
        if (cursor_index == -1) cursor_index = buffer.size() - 1;

        // 向前移动，最小为 0
        if (cursor_index > 0) cursor_index--;

        return true; // 只要按了，就进入回放模式
    }

    // --- 控制逻辑：下一张 ---
    void move_next() {
        std::lock_guard<std::mutex> lock(mtx);
        if (buffer.empty()) return;

        // 如果已经在实时模式，啥也不做
        if (cursor_index == -1) return;

        cursor_index++;

        // 如果游标追上了最新帧，恢复为实时模式 (-1)
        if (cursor_index >= buffer.size() - 1) {
            cursor_index = -1;
        }
    }

    // --- 控制逻辑：回到实时模式 ---
    void reset_to_live() {
        std::lock_guard<std::mutex> lock(mtx);
        cursor_index = -1;
    }

    // 获取当前状态文本（方便在界面显示）
    std::string get_status_text() {
        std::lock_guard<std::mutex> lock(mtx);
        if (buffer.empty()) return "Empty";
        if (cursor_index == -1) return "LIVE";
        return "REVIEW [" + std::to_string(cursor_index + 1) + "/" + std::to_string(buffer.size()) + "]";
    }
};