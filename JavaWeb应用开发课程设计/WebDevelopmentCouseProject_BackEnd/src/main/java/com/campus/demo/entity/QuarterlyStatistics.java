package com.campus.demo.entity;

import lombok.Data;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;

@Data
public class QuarterlyStatistics {

    private String quarter;
    private Double averageScore;
    private Integer assessedCount;
    private List<Map<String, Object>> trend = new ArrayList<>();
}
