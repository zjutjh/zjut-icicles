package com.campus.demo.dto;

import jakarta.validation.constraints.Max;
import jakarta.validation.constraints.Min;
import jakarta.validation.constraints.NotBlank;
import jakarta.validation.constraints.NotNull;
import lombok.Data;

@Data
public class CreateTaskProgressRequest {

    @NotNull(message = "progressRate 不能为空")
    @Min(value = 0, message = "progressRate 不能小于 0")
    @Max(value = 100, message = "progressRate 不能大于 100")
    private Integer progressRate;

    private String issueDesc;

    @NotBlank(message = "comment 不能为空")
    private String comment;
}
