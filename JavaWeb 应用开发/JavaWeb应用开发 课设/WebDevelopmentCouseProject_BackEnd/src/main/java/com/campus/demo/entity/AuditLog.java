package com.campus.demo.entity;

import com.campus.demo.enums.AuditResult;
import lombok.Data;

import java.time.LocalDateTime;

@Data
public class AuditLog {

    private Long id;
    private Long operatorId;
    private String operatorName;
    private String actionType;
    private String targetType;
    private String targetId;
    private AuditResult result;
    private LocalDateTime actionTime;
    private String detail;
}
