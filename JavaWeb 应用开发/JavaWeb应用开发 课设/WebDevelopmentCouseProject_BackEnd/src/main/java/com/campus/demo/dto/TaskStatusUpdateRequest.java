package com.campus.demo.dto;

import com.campus.demo.enums.TaskStatus;
import jakarta.validation.constraints.Max;
import jakarta.validation.constraints.Min;
import jakarta.validation.constraints.NotNull;
import lombok.Data;

@Data
public class TaskStatusUpdateRequest {

    @NotNull(message = "status 不能为空")
    private TaskStatus status;

    @Min(value = 0, message = "progressRate 不能小于 0")
    @Max(value = 100, message = "progressRate 不能大于 100")
    private Integer progressRate;

    private String issueDesc;
}
