package com.campus.demo.entity;

import com.baomidou.mybatisplus.annotation.IdType;
import com.baomidou.mybatisplus.annotation.TableField;
import com.baomidou.mybatisplus.annotation.TableId;
import com.baomidou.mybatisplus.annotation.TableName;
import com.campus.demo.enums.RoleCode;
import com.campus.demo.enums.UserStatus;
import lombok.Data;

import java.time.LocalDateTime;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

@Data
@TableName("sys_user")
public class SysUser {

    @TableId(type = IdType.AUTO)
    private Long id;

    private String username;

    private String realName;

    @TableField(exist = false)
    private String mobile;

    @TableField(exist = false)
    private String mobileMasked;

    @TableField(exist = false)
    private String email;

    @TableField(exist = false)
    private List<RoleCode> roleCodes = new ArrayList<>();

    @TableField(exist = false)
    private Long teamId;

    @TableField(exist = false)
    private Long memberId;

    @TableField(exist = false)
    private UserStatus status;

    @TableField(exist = false)
    private String passwordHash;

    @TableField(exist = false)
    private Boolean mustChangePassword;

    @TableField(exist = false)
    private LocalDateTime passwordExpireAt;

    @TableField(exist = false)
    private LocalDateTime lastLoginAt;

    @TableField(exist = false)
    private LocalDateTime createdAt;

    @TableField(exist = false)
    private LocalDateTime updatedAt;

    public RoleCode getRoleCode() {
        return roleCodes == null || roleCodes.isEmpty() ? null : roleCodes.get(0);
    }

    public void setRoleCode(RoleCode roleCode) {
        this.roleCodes = roleCode == null ? new ArrayList<>() : new ArrayList<>(Collections.singletonList(roleCode));
    }
}
