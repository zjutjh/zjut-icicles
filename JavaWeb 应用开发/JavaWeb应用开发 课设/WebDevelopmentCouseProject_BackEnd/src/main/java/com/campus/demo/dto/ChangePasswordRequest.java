package com.campus.demo.dto;

import jakarta.validation.constraints.NotBlank;
import lombok.Data;

@Data
public class ChangePasswordRequest {

    @NotBlank(message = "oldPassword 不能为空")
    private String oldPassword;

    @NotBlank(message = "newPassword 不能为空")
    private String newPassword;

    @NotBlank(message = "confirmPassword 不能为空")
    private String confirmPassword;
}
