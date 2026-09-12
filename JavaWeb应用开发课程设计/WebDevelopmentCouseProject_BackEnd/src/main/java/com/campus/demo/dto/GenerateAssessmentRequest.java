package com.campus.demo.dto;

import jakarta.validation.constraints.NotBlank;
import lombok.Data;

@Data
public class GenerateAssessmentRequest {

    @NotBlank(message = "assessMonth 不能为空")
    private String assessMonth;

    private Long teamId;
    private Boolean overwriteExisting = Boolean.FALSE;
}
