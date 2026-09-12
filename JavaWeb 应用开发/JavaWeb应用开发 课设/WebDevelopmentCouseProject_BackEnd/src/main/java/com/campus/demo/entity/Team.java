package com.campus.demo.entity;

import com.baomidou.mybatisplus.annotation.IdType;
import com.baomidou.mybatisplus.annotation.TableField;
import com.baomidou.mybatisplus.annotation.TableId;
import com.baomidou.mybatisplus.annotation.TableName;
import com.campus.demo.enums.TeamStatus;
import lombok.Data;

import java.time.LocalDateTime;

@Data
@TableName("team")
public class Team {

    @TableId(type = IdType.AUTO)
    private Long id;

    private String teamName;

    @TableField(exist = false)
    private Long managerId;

    @TableField(exist = false)
    private String managerName;

    private String departmentName;

    @TableField(exist = false)
    private String description;

    private TeamStatus status;

    @TableField(exist = false)
    private Integer memberCount;

    @TableField(exist = false)
    private LocalDateTime createdAt;

    @TableField(exist = false)
    private LocalDateTime updatedAt;

    public Long getManagerUserId() {
        return managerId;
    }

    public void setManagerUserId(Long managerUserId) {
        this.managerId = managerUserId;
    }
}
