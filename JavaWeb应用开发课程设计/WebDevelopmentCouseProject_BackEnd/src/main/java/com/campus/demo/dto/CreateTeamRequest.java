package com.campus.demo.dto;

import com.campus.demo.enums.TeamStatus;
import jakarta.validation.constraints.NotBlank;
import lombok.Data;

@Data
public class CreateTeamRequest {

    @NotBlank(message = "teamName 不能为空")
    private String teamName;

    @NotBlank(message = "departmentName 不能为空")
    private String departmentName;

    private Long managerId;

    private String description;

    private TeamStatus status = TeamStatus.ACTIVE;

    public Long getManagerUserId() {
        return managerId;
    }
}
