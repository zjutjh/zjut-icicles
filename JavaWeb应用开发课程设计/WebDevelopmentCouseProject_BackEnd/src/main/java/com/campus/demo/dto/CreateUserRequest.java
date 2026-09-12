package com.campus.demo.dto;

import com.campus.demo.enums.RoleCode;
import jakarta.validation.constraints.Email;
import jakarta.validation.constraints.NotBlank;
import jakarta.validation.constraints.NotEmpty;
import lombok.Data;

import java.util.List;

@Data
public class CreateUserRequest {

    @NotBlank(message = "username 不能为空")
    private String username;

    @NotBlank(message = "password 不能为空")
    private String password;

    @NotBlank(message = "realName 不能为空")
    private String realName;

    private String mobile;

    @Email(message = "email 格式不正确")
    private String email;

    @NotEmpty(message = "roleCodes 不能为空")
    private List<RoleCode> roleCodes;

    private Long teamId;

    private Long memberId;

    private Boolean mustChangePassword = Boolean.TRUE;

    public RoleCode getRoleCode() {
        return roleCodes == null || roleCodes.isEmpty() ? null : roleCodes.get(0);
    }
}
