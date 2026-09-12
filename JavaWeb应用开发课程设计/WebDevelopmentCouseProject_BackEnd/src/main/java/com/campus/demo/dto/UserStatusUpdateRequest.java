package com.campus.demo.dto;

import com.campus.demo.enums.UserStatus;
import jakarta.validation.constraints.NotNull;
import lombok.Data;

@Data
public class UserStatusUpdateRequest {

    @NotNull(message = "status 不能为空")
    private UserStatus status;

    private String reason;
}
