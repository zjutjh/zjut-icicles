package com.campus.demo.dto;

import jakarta.validation.constraints.NotBlank;
import lombok.Data;

@Data
public class UpdateWeeklyReportRequest {

    @NotBlank(message = "summary 不能为空")
    private String summary;

    @NotBlank(message = "nextPlan 不能为空")
    private String nextPlan;
}
