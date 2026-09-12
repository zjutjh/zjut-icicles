package com.campus.demo.dto;

import com.campus.demo.enums.ProjectStatus;
import jakarta.validation.constraints.NotNull;
import lombok.Data;

@Data
public class ProjectStatusUpdateRequest {

    @NotNull(message = "status 不能为空")
    private ProjectStatus status;

    private String comment;
}
