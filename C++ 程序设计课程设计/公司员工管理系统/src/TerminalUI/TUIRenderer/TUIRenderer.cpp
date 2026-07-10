#include "TUIRenderer.hpp"

/// @brief 更新页面
void TUIRenderer::updatePage() {
    // 获取终端宽度
    int screenWidth = std::max(80, getTerminalWidth());
    if (screenWidth != lastTerminalWidth) {
        // 终端宽度变化，清屏重绘
        std::cout << clearScreenCode;
        lastFrame.clear();
        lastTerminalWidth = screenWidth;
    }
    // 更新事件回调获取当前页面组件
    std::vector<std::reference_wrapper<TUICBase>> pageComponents = onUpdate ? onUpdate() : std::vector<std::reference_wrapper<TUICBase>>{};
    // 获取组件渲染输出
    std::vector<std::vector<TUIBlock>> blocks = {};
    for (int i = 0; i < pageComponents.size(); i++) {
        std::vector<std::vector<TUIBlock>> componentBlocks = pageComponents[i].get().Render(screenWidth);
        blocks.insert(blocks.end(), componentBlocks.begin(), componentBlocks.end());
    }
    renderBlocks(blocks);
}

/// @brief 将渲染单元渲染到终端
/// @param blocks
void TUIRenderer::renderBlocks(const std::vector<std::vector<TUIBlock>> &blocks) {
    // 重置光标
    std::cout << cursorResetCode;
    int cursorX = 0;
    std::pair<int, int> waitInputPos = {-1, -1};
    // 行间渲染循环
    for (int i = 0; i < blocks.size(); i++) {
        bool lineUpdate = false;       // 当前行是否需要更新
        std::ostringstream lineStream; // 行更新内容缓存
        int printStartX = -1;          // 行更新内容开始位置
        // 行内渲染循环
        for (int j = 0; j < blocks[i].size(); j++) {
            // 判断当前单元是否需要写入更新内容缓存
            // 如果行更新已为真 后续判断被短路不影响效率
            bool needsPrint = lineUpdate ||
                              lastFrame.size() <= i ||
                              lastFrame[i].size() <= j ||
                              blocks[i][j] != lastFrame[i][j];
            // 需要更新的块写入行更新内容缓存
            if (needsPrint) {
                if (printStartX == -1) {
                    printStartX = cursorX;
                }
                lineStream << blocks[i][j];
                lineUpdate = true;
            }
            // 记录输入控件位置(若有) 用于设置渲染结束时光标
            if (blocks[i][j].isInputBlock()) {
                waitInputPos = {i, cursorX};
            }
            cursorX += blocks[i][j].width();
        }
        // 打印行更新内容
        if (printStartX != -1) {
            std::printf(cursorMoveCode.data(), i + 1, printStartX + 1);
            std::cout << lineStream.str();
        }
        // 清除行尾多余内容
        std::printf(cursorMoveCode.data(), i + 1, cursorX + 1);
        std::cout << clearLineAfterCode << std::endl;
        cursorX = 0;
    }
    // 清除多余行
    std::cout << clearScreenAfterCode;
    // 如果存在输入控件 设置光标位置
    if (waitInputPos.first != -1) {
        std::cout << showCursorCode;
        std::printf(cursorMoveCode.data(), waitInputPos.first + 1, waitInputPos.second + 1);
    } else {
        std::cout << hideCursorCode;
    }
    // 更新帧记录
    lastFrame = blocks;
}

/// @brief 渲染循环
int TUIRenderer::renderLoop() {
    bool continueLoop = true;
    updatePage();
    while (continueLoop) {
        TUIEventType key = waitKey();
        if (onEvent) {
            onEvent(key);
        }
        updatePage();
    }
    return 0;
}