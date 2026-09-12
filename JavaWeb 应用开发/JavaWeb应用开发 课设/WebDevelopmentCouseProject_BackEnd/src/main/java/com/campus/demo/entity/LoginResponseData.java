package com.campus.demo.entity;

import lombok.AllArgsConstructor;
import lombok.Data;

@Data
@AllArgsConstructor
public class LoginResponseData {

    private String token;
    private String tokenType;
    private Integer expiresIn;
    private CurrentUser user;
}
