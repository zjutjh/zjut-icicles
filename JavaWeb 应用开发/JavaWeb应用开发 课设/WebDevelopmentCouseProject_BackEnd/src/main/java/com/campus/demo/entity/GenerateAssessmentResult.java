package com.campus.demo.entity;

import lombok.AllArgsConstructor;
import lombok.Data;

@Data
@AllArgsConstructor
public class GenerateAssessmentResult {

    private String assessMonth;
    private Integer generatedCount;
    private Integer skippedCount;
}
