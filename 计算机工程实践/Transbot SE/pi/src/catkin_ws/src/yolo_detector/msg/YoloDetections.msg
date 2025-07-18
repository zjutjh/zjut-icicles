# 多个目标检测结果
Header header                  # 时间戳和帧ID
YoloDetection[] detections     # 检测到的所有目标
int32 count                    # 检测到
string image_path              # 图像路径（用于与本地YOLO通信）
