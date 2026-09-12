package com.campus.demo.dto;

import jakarta.validation.constraints.NotBlank;
import lombok.Data;

@Data
public class ReviewWeeklyReportRequest {

    @NotBlank(message = "status 不能为空")
    private String status;

    private String managerComment;
}
