package com.campus.demo.dto;

import com.campus.demo.enums.RoleCode;
import jakarta.validation.constraints.Email;
import jakarta.validation.constraints.NotBlank;
import lombok.Data;

import java.util.List;

@Data
public class RegisterUserRequest {

    @NotBlank(message = "username cannot be blank")
    private String username;

    @NotBlank(message = "password cannot be blank")
    private String password;

    @NotBlank(message = "realName cannot be blank")
    private String realName;

    private String mobile;

    @Email(message = "email format is invalid")
    private String email;

    private List<RoleCode> roleCodes;

    private Long teamId;

    private Long memberId;

    private Boolean mustChangePassword = Boolean.FALSE;
}
