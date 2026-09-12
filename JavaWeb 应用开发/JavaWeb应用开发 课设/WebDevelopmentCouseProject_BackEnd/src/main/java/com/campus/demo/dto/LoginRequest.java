package com.campus.demo.dto;

import jakarta.validation.constraints.NotBlank;
import lombok.Data;

@Data
public class LoginRequest {

    @NotBlank(message = "username 不能为空")
    private String username;

    @NotBlank(message = "password 不能为空")
    private String password;
}
