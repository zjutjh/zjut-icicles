package com.campus.demo.entity;

import com.campus.demo.enums.RoleCode;
import lombok.Data;

import java.time.LocalDateTime;
import java.util.ArrayList;
import java.util.List;

@Data
public class CurrentUser {

    private Long userId;
    private String username;
    private String realName;
    private List<RoleCode> roleCodes = new ArrayList<>();
    private List<String> permissions = new ArrayList<>();
    private Long teamId;
    private String teamName;
    private Boolean mustChangePassword;
    private Boolean passwordExpired;
    private LocalDateTime lastLoginAt;
}
