package com.campus.demo.dto;

import com.campus.demo.enums.RoleCode;
import jakarta.validation.constraints.Max;
import jakarta.validation.constraints.Min;
import jakarta.validation.constraints.NotNull;
import lombok.Data;

@Data
public class ScoreAssessmentRequest {

    @NotNull(message = "scorerRole 不能为空")
    private RoleCode scorerRole;

    @NotNull(message = "rawScore 不能为空")
    @Min(value = 0, message = "rawScore 不能小于 0")
    @Max(value = 100, message = "rawScore 不能大于 100")
    private Double rawScore;

    private String comment;
}
