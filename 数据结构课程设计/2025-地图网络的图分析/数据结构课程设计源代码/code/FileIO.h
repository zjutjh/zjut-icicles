#pragma once
#include <string>
#include "Graph.h"

// 从文件加载地图数据到图中
bool loadMapFile(const std::string& fileName, Graph& graph);

// 将图的数据保存到文件中
bool saveMapFile(const std::string& fileName, const Graph& graph);
