package com.campus.demo.entity;

import lombok.Data;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;

@Data
public class YearlyStatistics {

    private String year;
    private Double averageScore;
    private Integer assessedCount;
    private List<Map<String, Object>> monthlyTrend = new ArrayList<>();
}
