package com.campus.demo.dto;

import com.campus.demo.enums.TeamStatus;
import jakarta.validation.constraints.NotBlank;
import jakarta.validation.constraints.NotNull;
import lombok.Data;

@Data
public class UpdateTeamRequest {

    @NotBlank(message = "teamName 不能为空")
    private String teamName;

    @NotBlank(message = "departmentName 不能为空")
    private String departmentName;

    private Long managerId;

    private String description;

    @NotNull(message = "status 不能为空")
    private TeamStatus status;
}
