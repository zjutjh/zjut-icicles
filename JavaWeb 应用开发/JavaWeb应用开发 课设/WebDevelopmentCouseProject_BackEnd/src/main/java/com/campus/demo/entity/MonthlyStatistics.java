package com.campus.demo.entity;

import lombok.Data;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;

@Data
public class MonthlyStatistics {

    private String month;
    private Double averageScore;
    private Double highestScore;
    private Double lowestScore;
    private Integer assessedCount;
    private List<Map<String, Object>> teamComparison = new ArrayList<>();
}
