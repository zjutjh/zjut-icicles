package com.campus.demo.dto;

import com.campus.demo.enums.PriorityLevel;
import com.campus.demo.enums.ProjectStatus;
import jakarta.validation.constraints.NotBlank;
import jakarta.validation.constraints.NotNull;
import lombok.Data;

import java.time.LocalDate;

@Data
public class CreateProjectRequest {

    @NotBlank(message = "projectName 不能为空")
    private String projectName;

    private Long ownerId;

    @NotNull(message = "status 不能为空")
    private ProjectStatus status;

    @NotNull(message = "priority 不能为空")
    private PriorityLevel priority;

    @NotNull(message = "startDate 不能为空")
    private LocalDate startDate;

    private LocalDate endDate;
    private String milestone;
    private String description;
}
