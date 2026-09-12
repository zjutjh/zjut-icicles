package com.campus.demo.dto;

import com.campus.demo.enums.MemberStatus;
import jakarta.validation.constraints.NotNull;
import lombok.Data;

@Data
public class MemberStatusUpdateRequest {

    @NotNull(message = "status 不能为空")
    private MemberStatus status;

    private String reason;
}
