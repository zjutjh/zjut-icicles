package com.campus.demo.dto;

import com.campus.demo.enums.PriorityLevel;
import jakarta.validation.constraints.NotBlank;
import jakarta.validation.constraints.NotNull;
import lombok.Data;

import java.time.LocalDate;

@Data
public class UpdateTaskRequest {

    @NotBlank(message = "title 不能为空")
    private String title;

    private Long projectId;

    @NotNull(message = "assigneeId 不能为空")
    private Long assigneeId;

    @NotNull(message = "priority 不能为空")
    private PriorityLevel priority;

    private LocalDate deadline;
    private LocalDate planStartDate;
    private LocalDate planEndDate;
    private String issueDesc;
}
