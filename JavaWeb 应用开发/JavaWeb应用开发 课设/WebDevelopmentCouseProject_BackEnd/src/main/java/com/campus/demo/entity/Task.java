package com.campus.demo.entity;

import com.campus.demo.enums.PriorityLevel;
import com.campus.demo.enums.TaskStatus;
import lombok.Data;

import java.time.LocalDate;

@Data
public class Task {

    private Long id;
    private String title;
    private Long projectId;
    private String projectName;
    private Long assigneeId;
    private String assigneeName;
    private PriorityLevel priority;
    private LocalDate deadline;
    private TaskStatus status;
    private Integer progressRate;
    private String weekNo;
    private LocalDate planStartDate;
    private LocalDate planEndDate;
    private String issueDesc;
}
