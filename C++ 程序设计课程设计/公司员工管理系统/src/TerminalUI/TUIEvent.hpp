#pragma once
enum class TUIEventType : int {
    KeyDelete = 8,
    KeyEnter = 13,
    KeyEsc = 27,
    // key events 0-255 reserved for valid characters
    KeyUp = 256,
    KeyDown,
    KeyLeft,
    KeyRight,
    Click,
    FocusGain,
    FocusLoss,
    Unknown = -1
};