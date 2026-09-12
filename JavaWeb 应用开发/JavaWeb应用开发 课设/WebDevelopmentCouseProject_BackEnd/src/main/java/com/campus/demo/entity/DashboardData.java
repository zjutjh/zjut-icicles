package com.campus.demo.entity;

import lombok.AllArgsConstructor;
import lombok.Data;

@Data
@AllArgsConstructor
public class DashboardData {

    private Double currentMonthAverageScore;
    private Integer pendingAssessments;
    private Integer overdueTasks;
    private Integer weeklyReportsPendingReview;
    private Integer activeProjects;
}
