package com.campus.demo.dto;

import jakarta.validation.constraints.NotBlank;
import jakarta.validation.constraints.NotNull;
import lombok.Data;

@Data
public class CreateWeeklyReportRequest {

    @NotNull(message = "memberId 不能为空")
    private Long memberId;

    @NotBlank(message = "weekNo 不能为空")
    private String weekNo;

    @NotBlank(message = "summary 不能为空")
    private String summary;

    @NotBlank(message = "nextPlan 不能为空")
    private String nextPlan;
}
