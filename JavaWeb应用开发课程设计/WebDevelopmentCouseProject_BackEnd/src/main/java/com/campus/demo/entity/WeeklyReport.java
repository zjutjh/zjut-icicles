package com.campus.demo.entity;

import com.campus.demo.enums.WeeklyReportStatus;
import lombok.Data;

import java.time.LocalDateTime;

@Data
public class WeeklyReport {

    private Long id;
    private Long memberId;
    private String memberName;
    private String weekNo;
    private String summary;
    private String nextPlan;
    private String managerComment;
    private WeeklyReportStatus status;
    private LocalDateTime createdAt;
    private LocalDateTime updatedAt;
}
